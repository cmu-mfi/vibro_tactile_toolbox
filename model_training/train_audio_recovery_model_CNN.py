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
import yaml

device = 'cuda' if torch.cuda.is_available() else 'cpu'
print('Using {} device'.format(device))

class VibrotactileDataset(Dataset):
  '''
  Prepare the Vibrotactile dataset for Prediction
  '''

  def __init__(self, dataset_type, channels, glob_path):

    self.total_length = 0
    num_channels = len(channels)

    paths = glob.glob(glob_path + 'fail/audio/*.png')
    
    if dataset_type == 'lego':
        paths = [path for path in paths if 'connection_failure' not in path]
        yaml_path = 'config/lego.yaml'
    else:
        paths = [path for path in paths if 'failure' not in path]
        yaml_path = 'config/nist.yaml'

    with open(yaml_path) as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            print(error)

    x_perturb_range = config['x_range']
    y_perturb_range = config['y_range']
    theta_perturb_range = config['theta_range']


    print(len(paths))
    self.total_length = int(len(paths) / 4) + 1
    print(self.total_length)

    self.X = torch.zeros([self.total_length,3*num_channels,201,221])
    self.y = torch.zeros([self.total_length,3])

    self.num_channels = num_channels

    current_trial = 0
    for path in paths:
      current_num = int(path[path.rfind('_')+1:-4])
      if current_num % 4 == 0:
          label = path[path.find('MoveDown')+len('MoveDown')+1:path.find('audio')-1]
          if label == 'fail':
            self.y[current_trial,0] = 1
          elif label == 'success':
            self.y[current_trial,1] = 1
          else:
            continue

          print(current_trial)
          perturbs = path[path.find('-p_')+3:]
          x_perturb = (float(perturbs[:perturbs.find('_')]) - x_perturb_range[0]) / (x_perturb_range[1] - x_perturb_range[0])
          perturbs = perturbs[perturbs.find('_')+1:]
          y_perturb = (float(perturbs[:perturbs.find('_')]) - y_perturb_range[0]) / (y_perturb_range[1] - y_perturb_range[0])
          perturbs = perturbs[perturbs.find('_')+1:]
          theta_perturb = (float(perturbs[:perturbs.find('_')]) - theta_perturb_range[0]) / (theta_perturb_range[1] - theta_perturb_range[0])
          self.y[current_trial,0] = x_perturb
          self.y[current_trial,1] = y_perturb
          self.y[current_trial,2] = theta_perturb

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
    total_elem = 201*221
    num_noise = int(1/20 * total_elem)
    
    for j in range(self.num_channels*3):
        aug_noise = np.zeros(201*221)
        idx = np.random.choice(len(aug_noise), num_noise, replace=False).astype(int)
        aug_noise[idx] = (np.random.random(num_noise) * 0.04) - 0.02

        img_input[j] += aug_noise.reshape(201,221)

    return img_input, self.y[i]


class CNNet_old(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(num_channels*3, 32, kernel_size=5)
        self.conv2 = nn.Conv2d(32, 16, kernel_size=5)
        self.conv2_drop = nn.Dropout2d()
        self.conv3 = nn.Conv2d(16, 8, kernel_size=5)
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(4032, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, 1)


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
        x = F.sigmoid(self.fc3(x))
        return x

class CNNet_try2(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(num_channels*3, 32, kernel_size=3)
        self.bn1 = nn.BatchNorm2d(32)
        self.conv2 = nn.Conv2d(32, 32, kernel_size=3)
        self.conv2_drop = nn.Dropout2d()
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 16, kernel_size=3)
        self.bn3 = nn.BatchNorm2d(16)
        self.conv4 = nn.Conv2d(16, 8, kernel_size=3)
        self.bn4 = nn.BatchNorm2d(8)
        #self.aap2d = nn.AdaptiveAvgPool2d(1)
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(81408, 1024)
        self.fc2 = nn.Linear(1024, 256)
        self.fc3 = nn.Linear(256, 2)


    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2_drop(self.conv2(x))))
        x = F.relu(self.bn3(self.conv3(x)))
        x = F.max_pool2d(F.relu(self.bn4(self.conv4(x))),2)
        #x = x.view(x.size(0), -1)
        x = self.flatten(x)
        x = self.fc1(x)
        x = F.dropout(x, training=self.training)
        x = self.fc2(x)
        x = F.dropout(x, training=self.training)
        x = self.fc3(x)
        return x

class CNNet(nn.Module):
    def __init__(self):
        super().__init__()
        model = models.resnet18()
        model.conv1 = nn.Conv2d(12, 64, kernel_size=(7, 7), stride=(2, 2), padding=(3, 3), bias=False)
        # new_module_list = list(model.modules())[:-1]
        model.fc = nn.Linear(in_features=512, out_features=3, bias=True)
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
min_test_loss = 2.0
def test(dataloader, model, type):
    global min_test_loss
    size = len(dataloader.dataset)
    model.eval()
    test_loss = 0

    with torch.no_grad():
        for batch, (X, Y) in enumerate(dataloader):
            X, Y = X.to(device), Y.to(device)
            pred = model(X)
            #print(pred.to('cpu').detach().numpy())
            test_loss += cost(pred, Y).item()

    if  (test_loss < min_test_loss):
        min_test_loss = test_loss
        model_scripted = torch.jit.script(model) # Export to TorchScript
        model_scripted.save('models/audio_recovery_'+type+'.pt') # Save
        print("====================== SAVED MODEL ==========================")

    print(f'\nTest loss: {test_loss:>8f}\n')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--type', '-t', type=str, default='lego')
    parser.add_argument('--train_ratio', '-r', type=float, default=0.8)
    args = parser.parse_args()

    channels = [0,1,2,3]
    num_channels = len(channels)

    if args.type == 'lego':
        paths = glob.glob(f'/home/mfi/Documents/vibrotactile_data/lego_dataset/*/*/test*/MoveDown/fail/audio/*.png')
    else:
        paths = glob.glob(f'/home/mfi/Documents/vibrotactile_data/nist_dataset/*/{args.type}/test*/MoveDown/fail/audio/*.png')
    if paths is not None:
        if args.type == 'lego':
            audio_train_dataset = VibrotactileDataset(args.type,channels,f'/home/mfi/Documents/vibrotactile_data/lego_dataset/*/*/vel*/MoveDown/')
            audio_test_dataset = VibrotactileDataset(args.type,channels, f'/home/mfi/Documents/vibrotactile_data/lego_dataset/*/*/test*/MoveDown/')
        else:
            audio_train_dataset = VibrotactileDataset(args.type,channels,f'/home/mfi/Documents/vibrotactile_data/nist_dataset/*/{args.type}/vel*/MoveDown/')
            audio_test_dataset = VibrotactileDataset(args.type,channels, f'/home/mfi/Documents/vibrotactile_data/nist_dataset/*/{args.type}/test*/MoveDown/')
    else:
        if args.type == 'lego':
            audio_dataset = VibrotactileDataset(args.type,channels, f'/home/mfi/Documents/vibrotactile_data/lego_dataset/*/{args.type}/vel*/MoveDown/')
        else:
            audio_dataset = VibrotactileDataset(args.type,channels, f'/home/mfi/Documents/vibrotactile_data/nist_dataset/*/{args.type}/vel*/MoveDown/')

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
    cost = torch.nn.MSELoss()

    # used to create optimal parameters
    learning_rate = 0.0001
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    epochs = 200

    for t in range(epochs):
        print(f'Epoch {t+1}\n-------------------------------')
        train(train_dataloader, model, cost, optimizer)
        test(test_dataloader, model, args.type)
    print('Done!')

    summary(model, input_size=(15, num_channels*3, 201, 221))

