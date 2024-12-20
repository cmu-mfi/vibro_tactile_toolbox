from torch.utils.data import DataLoader, Dataset
from torchvision import datasets, transforms
import torch
import torchaudio
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.data import Dataset, DataLoader
from torchvision import datasets, models, transforms
import torchvision
from torchinfo import summary
import pandas as pd
import os
import glob
from PIL import Image
import argparse
import cv2
import matplotlib.pyplot as plt


device = 'cuda' if torch.cuda.is_available() else 'cpu'
print('Using {} device'.format(device))

class VibrotactileDataset(Dataset):
  '''
  Prepare the Vibrotactile dataset for Prediction
  '''

  def __init__(self, channels, glob_path, block_type):

    self.total_length = 0
    num_channels = len(channels)

    paths = glob.glob(glob_path + 'success/audio/*.npy')
    paths += glob.glob(glob_path + 'fail/audio/*.npy')

    filtered_paths = []
    if block_type != '':
        for path in paths:
            if block_type not in path:
                filtered_paths.append(path)
        paths = filtered_paths.copy()            

    print(len(paths))
    self.total_length = int(len(paths) / 4) + 1
    print(self.total_length)

    self.X = torch.zeros([self.total_length,num_channels,256,87])
    self.y = torch.zeros([self.total_length,2])

    self.num_channels = num_channels

    current_trial = 0
    for path in paths:
      current_num = int(path[path.rfind('_')+1:-4])
      
      if current_num % 4 == 0:
          label = path[path.find('MoveDownToContact')+len('MoveDownToContact')+1:path.find('audio')-1]
          if label == 'fail':
            self.y[current_trial,0] = 1
          elif label == 'success':
            self.y[current_trial,1] = 1
          else:
            continue
          print(path)
          print(current_trial)

          channel_num = 0
          for channel in channels:
            #Load mel spectrogram
            spec = np.load(path[:path.rfind('_')+1]+str(current_num+channel)+'.npy')

            spec_tensor = torch.from_numpy(spec)
            self.X[current_trial,channel_num,:,:] = spec_tensor
            channel_num += 1
            
          current_trial += 1
    self.X = self.X.to(device)
    self.y = self.y.to(device)

  def __len__(self):
    return self.total_length

  def __getitem__(self, i):
    return self.X[i], self.y[i]


class CNNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(num_channels, 32, kernel_size=5)
        self.conv2 = nn.Conv2d(32, 16, kernel_size=5)
        self.conv2_drop = nn.Dropout2d()
        self.conv3 = nn.Conv2d(16, 8, kernel_size=5)
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(1568, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, 2)


    def forward(self, x):
        x = F.relu(F.max_pool2d(self.conv1(x), 2))
        x = F.relu(F.max_pool2d(self.conv2_drop(self.conv2(x)), 2))
        x = F.relu(F.max_pool2d(self.conv3(x), 2))
        #x = x.view(x.size(0), -1)
        x = self.flatten(x)
        x = F.relu(self.fc1(x))
        x = F.dropout(x, training=self.training)
        x = F.relu(self.fc2(x))
        x = F.dropout(x, training=self.training)
        x = self.fc3(x)
        return x


# Create the training function
def train(dataloader, model, loss, optimizer):
    model.train()
    size = len(dataloader.dataset)
    for batch, (X, Y) in enumerate(dataloader):

        #X, Y = X.to(device), Y.to(device)
        optimizer.zero_grad()
        pred = model(X)
        loss = cost(pred, Y)
        loss.backward()
        optimizer.step()

        if batch % 100 == 0:
            loss, current = loss.item(), batch * len(X)
            print(f'loss: {loss:>7f}  [{current:>5d}/{size:>5d}]')

# Create the validation/test function
max_test_correct = 0.75
min_test_loss = 0.05
def test(dataloader, model, type, save_suffix, block_type, proj_dir):
    global max_test_correct, min_test_loss
    size = len(dataloader.dataset)
    model.eval()
    test_loss, correct = 0, 0

    with torch.no_grad():
        for batch, (X, Y) in enumerate(dataloader):
            #X, Y = X.to(device), Y.to(device)
            pred = model(X)
            test_loss += cost(pred, Y).item()
            correct += (pred.argmax(1) == Y.argmax(1)).type(torch.float).sum().item()

    test_loss /= size
    correct /= size

    if correct > max_test_correct or (correct == max_test_correct and test_loss < min_test_loss):
        max_test_correct = correct
        min_test_loss = test_loss
        model_scripted = torch.jit.script(model) # Export to TorchScript
        if save_suffix == '': 
            if block_type == '':
                model_scripted.save(f'{proj_dir}/models/audio_outcome_{type}.pt') # Save
            else:
                model_scripted.save(f'{proj_dir}/models/audio_outcome_{type}_{block_type}.pt') # Save
        else: 
            model_scripted.save(f'{proj_dir}/models/channels/audio_outcome_{type}{save_suffix}.pt') # Save
        #torch.save(model, 'models/audio_outcome_'+type+'.pt')
        print("====================== SAVED MODEL ==========================")

    print(f'\nTest Error:\nacc: {(100*correct):>0.1f}%, avg loss: {test_loss:>8f}\n')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--type', '-t', type=str, default='lego')
    parser.add_argument('--train_ratio', '-r', type=float, default=0.8)
    parser.add_argument('--channels', '-c', type=str, default='')
    parser.add_argument('--block_type', '-b', type=str, default='')
    parser.add_argument('--data_dir', '-d', type=str, default='')
    parser.add_argument('--proj_dir', '-p', type=str, default='')
    args = parser.parse_args()
    
    if args.channels != '':
        channels_string_list = args.channels.split(',')
        channels = [ int(x) for x in channels_string_list]
        save_suffix = '_' + '_'.join(channels_string_list)
    else:
        channels = [0,1,2,3]
        save_suffix = ''

    num_channels = len(channels)

    if args.type == 'lego':
        paths = glob.glob(f'{args.data_dir}/lego_dataset/*/*/test*/MoveDownToContact/success/audio/*.npy')
    else:
        paths = glob.glob(f'{args.data_dir}/nist_dataset/*/{args.type}/test*/MoveDownToContact/success/audio/*.npy')
    if paths is not None and len(paths) > 0:
        if args.type == 'lego':
            audio_train_dataset = VibrotactileDataset(channels,f'{args.data_dir}/lego_dataset/*/*/vel*/MoveDownToContact/', args.block_type)
            audio_test_dataset = VibrotactileDataset(channels, f'{args.data_dir}/lego_dataset/*/*/test*/MoveDownToContact/', args.block_type)
        else:
            audio_train_dataset = VibrotactileDataset(channels,f'{args.data_dir}/nist_dataset/*/{args.type}/vel*/MoveDownToContact/', args.block_type)
            audio_test_dataset = VibrotactileDataset(channels, f'{args.data_dir}/nist_dataset/*/{args.type}/test*/MoveDownToContact/', args.block_type)
    else:
        if args.type == 'lego':
            audio_dataset = VibrotactileDataset(channels, f'{args.data_dir}/lego_dataset/*/*/vel*/MoveDownToContact/', args.block_type)
        else:
            audio_dataset = VibrotactileDataset(channels, f'{args.data_dir}/nist_dataset/*/{args.type}/vel*/MoveDownToContact/', args.block_type)
        print(len(audio_dataset))
        #split data to test and train
        #use 80% to train
        train_size = int(args.train_ratio * len(audio_dataset))
        test_size = len(audio_dataset) - train_size
        audio_train_dataset, audio_test_dataset = torch.utils.data.random_split(audio_dataset, [train_size, test_size])
      

    print("Training size:", len(audio_train_dataset))
    print("Testing size:",len(audio_test_dataset))


    train_dataloader = torch.utils.data.DataLoader(
        audio_train_dataset,
        batch_size=32,
        shuffle=True
    )

    test_dataloader = torch.utils.data.DataLoader(
        audio_test_dataset,
        batch_size=32,
        shuffle=True
    )

    model = CNNet().to(device)

    # cost function used to determine best parameters
    cost = torch.nn.CrossEntropyLoss()

    # used to create optimal parameters
    learning_rate = 0.0001
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    epochs = 1000

    for t in range(epochs):
        print(f'Epoch {t+1}\n-------------------------------')
        train(train_dataloader, model, cost, optimizer)
        test(test_dataloader, model, args.type, save_suffix, args.block_type, args.proj_dir)
    print('Done!')

    summary(model, input_size=(15, num_channels, 256, 87))
