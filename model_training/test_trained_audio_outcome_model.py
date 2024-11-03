from sklearn.metrics import confusion_matrix,ConfusionMatrixDisplay

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


device = 'cuda' if torch.cuda.is_available() else 'cpu'
print('Using {} device'.format(device))

class VibrotactileDataset(Dataset):
  '''
  Prepare the Vibrotactile dataset for Prediction
  '''

  def __init__(self, channels, glob_path):

    self.total_length = 0
    num_channels = len(channels)

    paths = glob.glob(glob_path + 'success/audio/*.npy')
    paths += glob.glob(glob_path + 'fail/audio/*.npy')
    #paths = glob.glob(glob_path+'*.npy')

    print(len(paths))
    self.total_length = int(len(paths) / 4) 
    print(self.total_length)

    self.X = torch.zeros([self.total_length,num_channels,256,87])
    self.y = torch.zeros([self.total_length])

    current_trial = 0
    for path in paths:
      current_num = int(path[path.rfind('_')+1:-4])
      if current_num % 4 == 0:
          print(current_trial)
          print(path)
          channel_num = 0
          # if 'fail' in path:
          #   self.y[current_trial] = 0
          # elif 'success' in path:
          #   self.y[current_trial] = 1
          label = path[path.find('MoveDown')+len('MoveDown')+1:path.find('audio')-1]
          if label == 'fail':
            self.y[current_trial] = 0
          elif label == 'success':
            self.y[current_trial] = 1

          for channel in channels:
            spec = np.load(path[:path.rfind('_')+1]+str(current_num+channel)+'.npy')
            spec_tensor = torch.from_numpy(spec)
            self.X[current_trial,channel_num,:,:] = spec_tensor
            channel_num += 1
            
          current_trial += 1
    self.X = self.X.to(device)

  def __len__(self):
    return self.total_length

  def __getitem__(self, i):
    return self.X[i], self.y[i]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--type', '-t', type=str, default='lego')
    parser.add_argument('--channels', '-c', type=str, default='')
    parser.add_argument('--block_type', '-b', type=str, default='')
    parser.add_argument('--data_dir', '-d', type=str, default='/home/mfi/Documents/vibrotactile_data')
    args = parser.parse_args()

    if args.channels != '':
        channels_string_list = args.channels.split(',')
        channels = [ int(x) for x in channels_string_list]
        save_suffix = '_' + '_'.join(channels_string_list)
    else:
        channels = [0,1,2,3]
        save_suffix = ''

    if args.type == 'lego':
      if args.block_type == '':
        audio_dataset = VibrotactileDataset(channels, f'{args.data_dir}/lego_dataset/*/*/test*/MoveDown/')
      else:
        audio_dataset = VibrotactileDataset(channels, f'{args.data_dir}/lego_dataset/*/{args.block_type}/*/MoveDown/')
    else:
        audio_dataset = VibrotactileDataset(channels, f'{args.data_dir}/nist_dataset/*/{args.type}/test*/MoveDown/')
    print(len(audio_dataset))

    test_dataloader = torch.utils.data.DataLoader(
        audio_dataset,
        batch_size=16,
        shuffle=False
    )

    if save_suffix == '': 
      if args.block_type != '':
        model = torch.jit.load('models/blocks/audio_outcome_'+args.type+'_'+args.block_type+'.pt') # load
      else:
        model = torch.jit.load('models/audio_outcome_'+args.type+'.pt') # load
    else: 
        model = torch.jit.load('models/channels/audio_outcome_'+args.type+save_suffix+'.pt') # load

    model.eval()
    model.to(device)

    class_map = ['fail', 'success']

    y_true = []
    y_pred = []

    with torch.no_grad():
        for batch, (X, Y) in enumerate(test_dataloader):
            #X = X.to(device)
            pred = model(X)
            y_true.extend(list(Y.detach().numpy()))
            y_pred.extend(list(pred.argmax(1).to('cpu').detach().numpy()))
            #print(list(pred.to('cpu').detach().numpy()))

    #f, ax = plt.subplots()
    #ax.set_title('Vibrotactile Audio Confusion Matrix for '+args.type)
    #_ = ConfusionMatrixDisplay.from_predictions(y_true, y_pred, display_labels=np.array(class_map), ax=ax)

    #plt.show()
    cfm = confusion_matrix(y_true, y_pred)
    print(cfm)
    print(np.count_nonzero(np.equal(y_true, y_pred)) / len(y_true))
