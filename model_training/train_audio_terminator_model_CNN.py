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

  def __init__(self, channels, glob_path):

    self.total_length = 0
    num_channels = len(channels)

    paths = glob.glob(glob_path + 'nominal/audio/*.png')
    paths += glob.glob(glob_path + 'terminate/audio/*.png')

    print(len(paths))
    self.total_length = int(len(paths) / 4) + 1
    print(self.total_length)

    self.X = torch.zeros([self.total_length,3*num_channels,201,111])
    self.y = torch.zeros([self.total_length,2])

    self.num_channels = num_channels

    current_trial = 0
    for path in paths:
      current_num = int(path[path.rfind('_')+1:-4])
      if current_num % 4 == 0:
          label = path[path.find('MoveDown')+len('MoveDown')+1:path.find('audio')-1]
          if label == 'nominal':
            self.y[current_trial,0] = 1
          elif label == 'terminate':
            self.y[current_trial,1] = 1
          else:
            continue

          print(current_trial)

          channel_num = 0
          for channel in channels:
            #Load image by OpenCV
            #print(current_num+channel)
            cv_image = cv2.imread(path[:path.rfind('_')+1]+str(current_num+channel)+'.png')

            #Convert img to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb_image)
            image_tensor = transforms.ToTensor()(pil_image)
            self.X[current_trial,channel_num*3:(channel_num+1)*3,:,:] = image_tensor
            channel_num += 1
            
          current_trial += 1

  def __len__(self):
    return self.total_length

  def __getitem__(self, i):
    img_input = self.X[i]
    #print(img_input)
    total_elem = 201*111
    num_noise = int(1/20 * total_elem)
    
    for j in range(self.num_channels*3):
        aug_noise = np.zeros(201*111)
        idx = np.random.choice(len(aug_noise), num_noise, replace=False).astype(int)
        aug_noise[idx] = (np.random.random(num_noise) * 0.04) - 0.02

        img_input[j] += aug_noise.reshape(201,111)

    return img_input, self.y[i]

class CNNet(nn.Module):
    def __init__(self):
        super().__init__()
        model = models.resnet18()
        model.conv1 = nn.Conv2d(12, 64, kernel_size=(7, 7), stride=(2, 2), padding=(3, 3), bias=False)
        # new_module_list = list(model.modules())[:-1]
        model.fc = nn.Linear(in_features=512, out_features=2, bias=True)
        self.model = model #nn.Sequential(*new_module_list)
        # self.fc = nn.Linear(512, 2)

    def forward(self, x):
        # import pdb;pdb.set_trace()
        x = self.model(x)
        # x = self.fc(x)
        return x

# Create the training function
def train(dataloader, model, loss, optimizer):
    model.train()
    size = len(dataloader.dataset)
    for batch, (X, Y) in enumerate(dataloader):

        X, Y = X.to(device), Y.to(device)
        optimizer.zero_grad()
        pred = model(X)
        loss = cost(pred, Y)
        loss.backward()
        optimizer.step()

        if batch % 100 == 0:
            loss, current = loss.item(), batch * len(X)
            print(f'loss: {loss:>7f}  [{current:>5d}/{size:>5d}]')

# Create the validation/test function
max_test_correct = 0.85
min_test_loss = 0.05
def test(dataloader, model, type):
    global max_test_correct, min_test_loss
    size = len(dataloader.dataset)
    model.eval()
    test_loss, correct = 0, 0

    with torch.no_grad():
        for batch, (X, Y) in enumerate(dataloader):
            X, Y = X.to(device), Y.to(device)
            pred = model(X)
            test_loss += cost(pred, Y).item()
            correct += (pred.argmax(1) == Y.argmax(1)).type(torch.float).sum().item()

    test_loss /= size
    correct /= size

    if correct > max_test_correct or (correct == max_test_correct and test_loss < min_test_loss):
        max_test_correct = correct
        min_test_loss = test_loss
        model_scripted = torch.jit.script(model) # Export to TorchScript
        model_scripted.save('models/audio_terminator_'+type+'.pt') # Save
        print("====================== SAVED MODEL ==========================")

    print(f'\nTest Error:\nacc: {(100*correct):>0.1f}%, avg loss: {test_loss:>8f}\n')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--type', '-t', type=str, default='lego')
    parser.add_argument('--train_ratio', '-r', type=float, default=0.8)
    args = parser.parse_args()

    channels = [0,1,2,3]
    num_channels = len(channels)

    if args.type == 'lego':
        paths = glob.glob(f'/home/mfi/Documents/vibrotactile_data/lego_dataset/*/*/test*/MoveDown/nominal/audio/*.png')
    else:
        paths = glob.glob(f'/home/mfi/Documents/vibrotactile_data/nist_dataset/*/{args.type}/test*/MoveDown/nominal/audio/*.png')
    if paths is not None and len(paths) > 0:
        if args.type == 'lego':
            audio_train_dataset = VibrotactileDataset(channels,f'/home/mfi/Documents/vibrotactile_data/lego_dataset/*/*/vel*/MoveDown/')
            audio_test_dataset = VibrotactileDataset(channels, f'/home/mfi/Documents/vibrotactile_data/lego_dataset/*/*/test*/MoveDown/')
        else:
            audio_train_dataset = VibrotactileDataset(channels,f'/home/mfi/Documents/vibrotactile_data/nist_dataset/*/{args.type}/vel*/MoveDown/')
            audio_test_dataset = VibrotactileDataset(channels, f'/home/mfi/Documents/vibrotactile_data/nist_dataset/*/{args.type}/test*/MoveDown/')
    else:
        if args.type == 'lego':
            audio_dataset = VibrotactileDataset(channels, f'/home/mfi/Documents/vibrotactile_data/lego_dataset/*/{args.type}/vel*/MoveDown/')
        else:
            audio_dataset = VibrotactileDataset(channels, f'/home/mfi/Documents/vibrotactile_data/nist_dataset/*/{args.type}/vel*/MoveDown/')
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
        batch_size=64,
        shuffle=True
    )

    test_dataloader = torch.utils.data.DataLoader(
        audio_test_dataset,
        batch_size=64,
        shuffle=True
    )

    model = CNNet().to(device)

    # cost function used to determine best parameters
    cost = torch.nn.CrossEntropyLoss()

    # used to create optimal parameters
    learning_rate = 0.0001
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    epochs = 200

    for t in range(epochs):
        print(f'Epoch {t+1}\n-------------------------------')
        train(train_dataloader, model, cost, optimizer)
        test(test_dataloader, model, args.type)
    print('Done!')

    summary(model, input_size=(15, num_channels*3, 201, 111))
