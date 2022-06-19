#! /usr/bin/python3

import cv2
from cv2 import transform
import torch.utils.data as data
from localbot_core.src.utilities import read_pcd, matrixToXYZ, matrixToQuaternion
from localbot_localization.src.utilities import normalize_quat
import numpy as np
import torch
import os
import yaml
from yaml.loader import SafeLoader
from PIL import Image as ImagePIL
import pandas as pd
from torchvision import transforms

class DatasetPSA(data.Dataset):
    def __init__(self, transforms):
        self.folder = '/home/danc/Desktop/psa_1306_aula'
        self.transforms = transforms
        self.csv = pd.read_csv(f'{self.folder}/data.csv')
        
        
    def __getitem__(self, index):
        
        # load image
        image = ImagePIL.open(f'{self.folder}/{index:05d}.jpg')
        image = self.transforms(image)
        
        # load steering
        
        steering = self.csv.iloc[index][1].astype(np.float32)
        

        return image, steering
    

    def __len__(self):
        return self.csv.shape[0]



def main():
    
    rgb_transforms = transforms.Compose([
        transforms.Resize((66,200)),
        transforms.ToTensor(),
        transforms.Normalize([0.19054175422901043, 0.19067991892465383, 0.19061099275088064], [0.3224361491843105, 0.3223948440593156, 0.32235567748029115])
    ])


    dataset = DatasetPSA(transforms=rgb_transforms)
    
    
    print(dataset[100])


if __name__ == '__main__':
    main()
    
