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
from torch.utils.data import Dataset, DataLoader
from torchvision import datasets, models, transforms
from torchinfo import summary
import pandas as pd
import os

import matplotlib.pyplot as plt


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

from collections import Counter

# labels in training set
test_classes = [label for _, label in audio_dataset]
Counter(test_classes)

test_dataloader = torch.utils.data.DataLoader(
    audio_dataset,
    batch_size=16,
    num_workers=2,
    shuffle=False
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

model = CNNet()
model = torch.load('./models/audio_outcome_lego.pt')
model.eval()
model.to(device)

class_map = ['nominal', 'error']

y_true = []
y_pred = []


with torch.no_grad():
    for batch, (X, Y) in enumerate(test_dataloader):
        X = X.to(device)
        pred = model(X)
        y_true.extend(list(Y.detach().numpy()))
        print()
        y_pred.extend(list(pred.argmax(1).to('cpu').detach().numpy()))
        print(np.subtract(pred.argmax(1).to('cpu').detach().numpy(),Y.detach().numpy()))

f, ax = plt.subplots()
ax.set_title('Vibrotactile Audio Confusion Matrix')
_ = ConfusionMatrixDisplay.from_predictions(y_true, y_pred, display_labels=np.array(['fail','success']), ax=ax)

plt.show()