# Copyright 2024 RobotAlert, Inc., All Rights Reserved
# Proprietary and confidential. Unauthorized copying of this file via any medium is strictly prohibited.
# RobotAlert, Inc. is Kevin Zhang (klz1@andew.cmu.edu), Rohit Sonker (rsonker@andrew.cmu.edu)], and Tianqin Li (tianqinl@andrew.cmu.edu)

from torch.utils.data import DataLoader
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
from torchinfo import summary
import pandas as pd
import os

device = 'cuda' if torch.cuda.is_available() else 'cpu'
print('Using {} device'.format(device))

data_path = '/mnt/hdd1/vibrotactile_data/lego_dataset/train/' #looking in subfolder train

audio_dataset = datasets.ImageFolder(
    root=data_path,
    transform=transforms.Compose([
                                  transforms.ToTensor()
                                  ])
)
print(audio_dataset)


class_map=audio_dataset.class_to_idx

print("\nClass category and index of the images: {}\n".format(class_map))

#split data to test and train
#use 80% to train
train_size = int(0.85 * len(audio_dataset))
test_size = len(audio_dataset) - train_size
audio_train_dataset, audio_test_dataset = torch.utils.data.random_split(audio_dataset, [train_size, test_size])

print("Training size:", len(audio_train_dataset))
print("Testing size:",len(audio_test_dataset))

from collections import Counter

# labels in training set
train_classes = [label for _, label in audio_train_dataset]
Counter(train_classes)


train_dataloader = torch.utils.data.DataLoader(
    audio_train_dataset,
    batch_size=15,
    num_workers=2,
    shuffle=True
)

test_dataloader = torch.utils.data.DataLoader(
    audio_test_dataset,
    batch_size=15,
    num_workers=2,
    shuffle=True
)

class CNNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size=5)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5)
        self.conv2_drop = nn.Dropout2d()
        self.conv3 = nn.Conv2d(32, 16, kernel_size=5)
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(8064, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, 3)


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
        x = F.relu(self.fc3(x))
        return F.log_softmax(x,dim=1)

model = CNNet().to(device)

# cost function used to determine best parameters
cost = torch.nn.CrossEntropyLoss()

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

def test(dataloader, model):
    global max_test_correct
    size = len(dataloader.dataset)
    model.eval()
    test_loss, correct = 0, 0

    with torch.no_grad():
        for batch, (X, Y) in enumerate(dataloader):
            print(X.size())
            X, Y = X.to(device), Y.to(device)
            pred = model(X)

            test_loss += cost(pred, Y).item()
            correct += (pred.argmax(1)==Y).type(torch.float).sum().item()

    test_loss /= size
    correct /= size

    if correct > max_test_correct:
        max_test_correct = correct
        torch.save(model, 'models/audio_outcome_lego.pt')
        print("====================== SAVED MODEL ==========================")

    print(f'\nTest Error:\nacc: {(100*correct):>0.1f}%, avg loss: {test_loss:>8f}\n')

epochs = 200

for t in range(epochs):
    print(f'Epoch {t+1}\n-------------------------------')
    train(train_dataloader, model, cost, optimizer)
    test(test_dataloader, model)
print('Done!')

summary(model, input_size=(15, 3, 221, 201))

model.eval()
test_loss, correct = 0, 0
class_map = ['nominal', 'error']
print("MAX TEST CORRECT: " + str(max_test_correct))

with torch.no_grad():
    for batch, (X, Y) in enumerate(test_dataloader):
        X, Y = X.to(device), Y.to(device)
        pred = model(X)
        print("Predicted:\nvalue={}, class_name= {}\n".format(pred[0].argmax(0),class_map[pred[0].argmax(0)]))
        print("Actual:\nvalue={}, class_name= {}\n".format(Y[0],class_map[Y[0]]))
        break
