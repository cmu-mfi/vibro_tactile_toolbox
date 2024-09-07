from sklearn.metrics import ConfusionMatrixDisplay

from torch.utils.data import DataLoader
from torchvision import datasets, transforms
import torch
import torchaudio
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.data import Dataset, DataLoader, Dataset
import torchvision
from torchvision import datasets, models, transforms
from torchinfo import summary
import pandas as pd
import os
import glob
import argparse
import matplotlib.pyplot as plt


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
        self.X[current_trial,channel_num*4:(channel_num+1)*4] = torch_image #transforms.ToTensor()(pil_image).unsqueeze_(0)
        
      current_trial += 1

      self.X.to(device)
      self.y.to(device)

  def __len__(self):
    return self.total_length

  def __getitem__(self, i):
    return self.X[i], self.y[i]

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

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--type', '-t', type=str, default='lego')
    args = parser.parse_args()

    channels = [0,1,2,3]
    num_channels = len(channels)
    audio_dataset = VibrotactileDataset(args.type,channels)
    print(len(audio_dataset))

    test_dataloader = torch.utils.data.DataLoader(
        audio_dataset,
        batch_size=16,
        num_workers=2,
        shuffle=False
    )

    model = CNNet()
    model = torch.load('./models/audio_outcome_'+args.type+'.pt')
    model.eval()
    model.to(device)

    class_map = ['fail', 'success']

    y_true = []
    y_pred = []

    with torch.no_grad():
        for batch, (X, Y) in enumerate(test_dataloader):
            X = X.to(device)
            pred = model(X)
            y_true.extend(list(Y.detach().numpy()))
            y_pred.extend(list(pred.round().to('cpu').detach().numpy()))

    f, ax = plt.subplots()
    ax.set_title('Vibrotactile Audio Confusion Matrix for '+args.type)
    _ = ConfusionMatrixDisplay.from_predictions(y_true, y_pred, display_labels=np.array(class_map), ax=ax)

    plt.show()