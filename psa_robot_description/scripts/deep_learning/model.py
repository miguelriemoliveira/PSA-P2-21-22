#! /usr/bin/python3

import torch
import torch.nn as nn
import torch.nn.parallel
import torch.utils.data
import numpy as np
import torch.nn.functional as F

class CNN(nn.Module):
    def __init__(self):
        super(CNN, self).__init__()
        
        self.conv1 = nn.Conv2d(in_channels=3, out_channels=24, kernel_size=5, stride=2, padding=0)
        self.conv2 = nn.Conv2d(in_channels=24, out_channels=36, kernel_size=5, stride=2, padding=0)
        self.conv3 = nn.Conv2d(in_channels=36, out_channels=48, kernel_size=5, stride=2, padding=0)
        self.conv4 = nn.Conv2d(in_channels=48, out_channels=64, kernel_size=3, stride=1, padding=0)
        self.conv5 = nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=1, padding=0)
        
        self.fc1 = nn.Linear(1152, 100)
        self.fc2 = nn.Linear(100, 50)
        self.fc3 = nn.Linear(50, 10)
        self.fc4 = nn.Linear(10, 1)
        

    # instead of treating the relu as modules, we can treat them as functions. We can access them via torch funtional
    def forward(self, x, verbose=False):  # this is where we pass the input into the module

        if verbose: print('shape ' + str(x.shape))
        x = F.relu(self.conv1(x))
        if verbose: print('shape ' + str(x.shape))
        x = F.relu(self.conv2(x))
        if verbose: print('shape ' + str(x.shape))
        x = F.relu(self.conv3(x))
        if verbose: print('shape ' + str(x.shape))
        x = F.relu(self.conv4(x))
        if verbose: print('shape ' + str(x.shape))
        x = F.relu(self.conv5(x))
        if verbose: print('shape ' + str(x.shape))
        x = x.view(x.size(0), -1)
        
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = self.fc4(x)
        return x



