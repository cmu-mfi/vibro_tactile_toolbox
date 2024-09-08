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
from PIL import Image
import cv2


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
        paths = glob.glob('/mnt/hdd1/vibrotactile_data/lego_dataset/*/*/*/MoveDown/*/audio/*.png')
    elif dataset_type == 'dsub':
        paths = glob.glob('/mnt/hdd1/vibrotactile_data/nist_dataset/*/dsub/test_vel*/MoveDown/*/audio/*.png')
    elif dataset_type == 'waterproof':
        paths = glob.glob('/mnt/hdd1/vibrotactile_data/nist_dataset/*/waterproof/test_vel*/MoveDown/*/audio/*.png')

    print(len(paths))
    self.total_length = int(len(paths) / 4) + 1
    print(self.total_length)

    self.X = torch.zeros([self.total_length,3*num_channels,201,221])
    self.y = torch.zeros([self.total_length])

    current_trial = 0
    for path in paths:
      current_num = int(path[path.rfind('_')+1:-4])
      if current_num % 4 == 0:
          print(current_trial)
          channel_num = 0
          label = path[path.find('MoveDown')+len('MoveDown')+1:path.find('audio')-1]
          if label == 'fail':
            self.y[current_trial] = 0
          elif label == 'success':
            self.y[current_trial] = 1

          for channel in channels:
            #Load image by OpenCV
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
    return self.X[i], self.y[i]

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

    model = torch.jit.load('./models/audio_outcome_'+args.type+'.pt')
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
            y_pred.extend(list(pred.argmax(1).to('cpu').detach().numpy()))
            print(list(pred.to('cpu').detach().numpy()))

    f, ax = plt.subplots()
    ax.set_title('Vibrotactile Audio Confusion Matrix for '+args.type)
    _ = ConfusionMatrixDisplay.from_predictions(y_true, y_pred, display_labels=np.array(class_map), ax=ax)

    plt.show()