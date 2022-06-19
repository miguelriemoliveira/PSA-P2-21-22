#!/usr/bin/env python3

import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from colorama import Fore, Style
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime  # to track the time each epoch takes
import argparse
import sys
import os
import yaml
from yaml.loader import SafeLoader
from colorama import Fore
from dataset import DatasetPSA
from model import CNN
from torchvision import transforms

def main():
   
    cuda_available = torch.cuda.is_available()
   
    rgb_transforms = transforms.Compose([
        transforms.Resize((66,200)),
        transforms.ToTensor(),
        transforms.Normalize([0.19054175422901043, 0.19067991892465383, 0.19061099275088064], [0.3224361491843105, 0.3223948440593156, 0.32235567748029115])
    ])
    
    train_dataset = DatasetPSA(transforms=rgb_transforms)
    

    batch_size = 64
    
    train_loader = torch.utils.data.DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True)
    

    model = CNN()

    # ----------------------------------
    # Define loss function
    # ----------------------------------
    criterion = nn.MSELoss()
    

    optimizer = optim.Adam(params = model.parameters(), lr=1e-4, weight_decay=1e-3) # the most common optimizer in DL
    # weight decay is the L2 regularizer!!

   

    # ----------------------------------
    # Training
    # ----------------------------------

    n_epochs = 100

    if cuda_available:
        model.cuda()  # Send model to GPU
        criterion.cuda()  # Send criterion to GPU
        
    train_losses= []
    for epoch in range(0, n_epochs):
        t0 = datetime.now()
        train_losses_per_batch = []

        for image, steering in train_loader:  # iterate batches
            
            
            
            steering = steering.view(steering.size(0), 1)
            
            if cuda_available:
                image, steering =  image.cuda(), steering.cuda()  # move data into GPU
                
            optimizer.zero_grad()  # Clears the gradients of all optimized tensors (always needed in the beginning of the training loop)

            model = model.train()  # Sets the module in training mode. For example, the dropout module can only be use in training mode.

            predicted_steering = model(image)  # our model outputs the pose, and the transformations used
            

            

            loss = criterion(predicted_steering, steering)

            loss.backward()  # Computes the gradient of current tensor w.r.t. graph leaves.
            optimizer.step()  # Performs a single optimization step (parameter update).

            train_losses_per_batch.append(loss.item())

        train_loss_epoch = float(np.mean(train_losses_per_batch))

        
        dt = datetime.now() - t0
        print(f'epoch {epoch + 1}/{n_epochs}, train_loss: {train_loss_epoch:.4f}, duration: {dt}')
        

    
    torch.save(model.state_dict(), '/home/danc/Desktop/model_psa_1306_aula2/model.pth')
    

if __name__ == "__main__":
    main()
