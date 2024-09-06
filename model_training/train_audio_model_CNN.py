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

device = 'cuda' if torch.cuda.is_available() else 'cpu'
print('Using {} device'.format(device))

class VibrotactileDataset(Dataset):
  '''
  Prepare the Vibrotactile dataset for Prediction
  '''

  def __init__(self, dataset_type, channels):

    self.total_length = 0
    num_channels = len(channels)

    if dataset_type == 'lego':
        paths = glob.glob('/mnt/hdd1/vibrotactile_data/lego_dataset/*/*/*/MoveDown/*/audio/*_0.png')
    elif dataset_type == 'dsub':
        paths = glob.glob('/mnt/hdd1/vibrotactile_data/nist_dataset/*/dsub/*/MoveDown/*/audio/*_0.png')
    elif dataset_type == 'waterproof':
        paths = glob.glob('/mnt/hdd1/vibrotactile_data/nist_dataset/*/waterproof/*/MoveDown/*/audio/*_0.png')

    self.total_length = len(paths)

    self.X = torch.zeros([self.total_length,4*num_channels,201,221])
    self.y = torch.zeros([self.total_length,1])

    current_trial = 0
    for path in paths:
      print(current_trial)
      channel_num = 0
      label = path[path.find('MoveDown')+len('MoveDown')+1:path.find('audio')-1]
      if label == 'fail':
        self.y[current_trial] = 0
      elif label == 'success':
        self.y[current_trial] = 1

      for channel in channels:
        torch_image = torchvision.io.read_image(path[:-5]+str(channel)+'.png')
        #print(torch_image.shape)
        #pil_image = Image.open(path[:-5]+str(channel)+'.png')
        self.X[current_trial,channel_num*4:(channel_num+1)*4] = torch_image #transforms.ToTensor()(pil_image).unsqueeze_(0)
        
      current_trial += 1

      self.X.to(device)
      self.y.to(device)

  def __len__(self):
    return self.total_length

  def __getitem__(self, i):
    return self.X[i], self.y[i]

channels = [0,1,2,3]
num_channels = len(channels)
audio_dataset = VibrotactileDataset('lego',channels)
print(len(audio_dataset))
#split data to test and train
#use 80% to train
train_size = int(0.85 * len(audio_dataset))
test_size = len(audio_dataset) - train_size
audio_train_dataset, audio_test_dataset = torch.utils.data.random_split(audio_dataset, [train_size, test_size])

print("Training size:", len(audio_train_dataset))
print("Testing size:",len(audio_test_dataset))

# from collections import Counter

# # labels in training set
# train_classes = [label for _, label in audio_train_dataset]
# Counter(train_classes)


train_dataloader = torch.utils.data.DataLoader(
    audio_train_dataset,
    batch_size=16,
    num_workers=2,
    shuffle=True
)

test_dataloader = torch.utils.data.DataLoader(
    audio_test_dataset,
    batch_size=16,
    num_workers=2,
    shuffle=True
)

class CNNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(num_channels*4, 16, kernel_size=5)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5)
        self.conv2_drop = nn.Dropout2d()
        self.conv3 = nn.Conv2d(32, 16, kernel_size=5)
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(8064, 512)
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

model = CNNet().to(device)

# cost function used to determine best parameters
cost = torch.nn.BCELoss()

# used to create optimal parameters
learning_rate = 0.0001
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

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

def test(dataloader, model):
    global max_test_correct, min_test_loss
    size = len(dataloader.dataset)
    model.eval()
    test_loss, correct = 0, 0

    with torch.no_grad():
        for batch, (X, Y) in enumerate(dataloader):
            X, Y = X.to(device), Y.to(device)
            pred = model(X)

            test_loss += cost(pred, Y).item()
            correct += (pred.round() == Y).type(torch.float).sum().item()

    test_loss /= size
    correct /= size

    if correct >= max_test_correct and test_loss < min_test_loss:
        max_test_correct = correct
        min_test_loss = test_loss
        torch.save(model, 'models/audio_outcome_lego.pt')
        print("====================== SAVED MODEL ==========================")

    print(f'\nTest Error:\nacc: {(100*correct):>0.1f}%, avg loss: {test_loss:>8f}\n')

epochs = 200

for t in range(epochs):
    print(f'Epoch {t+1}\n-------------------------------')
    train(train_dataloader, model, cost, optimizer)
    test(test_dataloader, model)
print('Done!')

summary(model, input_size=(15, num_channels*4, 201, 221))
