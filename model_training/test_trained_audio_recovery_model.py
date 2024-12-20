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
from PIL import Image
import cv2
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

    paths = glob.glob(glob_path + 'fail/audio/*.npy')

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
    self.total_length = int(len(paths) / 4)
    print(self.total_length)

    self.X = torch.zeros([self.total_length,num_channels,256,87])
    self.y = torch.zeros([self.total_length,3])

    current_trial = 0
    for path in paths:
      current_num = int(path[path.rfind('_')+1:-4])
      if current_num % 4 == 0:
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
            spec = np.load(path[:path.rfind('_')+1]+str(current_num+channel)+'.npy')

            spec_tensor = torch.from_numpy(spec)
            self.X[current_trial,channel_num,:,:] = spec_tensor
            channel_num += 1
            
          current_trial += 1

  def __len__(self):
    return self.total_length

  def __getitem__(self, i):
    return self.X[i], self.y[i]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--type', '-t', type=str, default='lego')
    parser.add_argument('--data_dir', '-d', type=str, default='')
    parser.add_argument('--proj_dir', '-p', type=str, default='')
    args = parser.parse_args()

    channels = [0,1,2,3]
    num_channels = len(channels)
    if args.type == 'lego':
        audio_dataset = VibrotactileDataset(args.type,channels, f'{args.data_dir}/lego_dataset/*/*/test*/MoveDown/')
    else:
        audio_dataset = VibrotactileDataset(args.type,channels, f'{args.data_dir}/nist_dataset/*/{args.type}/test*/MoveDown/')
    print(len(audio_dataset))

    test_dataloader = torch.utils.data.DataLoader(
        audio_dataset,
        batch_size=16,
        shuffle=False
    )

    model = torch.jit.load(f'{args.proj_dir}/models/audio_recovery_{args.type}.pt')
    model.eval()
    model.to(device)

    y_true = []
    y_pred = []

    with torch.no_grad():
        for batch, (X, Y) in enumerate(test_dataloader):
            X = X.to(device)
            pred = model(X)
            y_true.extend(list(Y.detach().numpy()))
            y_pred.extend(list(pred.to('cpu').detach().numpy()))
    RMSE = np.mean(np.sum(np.sqrt(np.square(np.subtract(y_true,y_pred))),axis=1))
    print(RMSE)
