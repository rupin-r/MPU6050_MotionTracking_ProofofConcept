import os
import json
import torch
import numpy as np
from torch.utils.data import Dataset
from torchvision import transforms
import ai8x  # Assuming you have this for normalization as in the original code

class IMU_AI(Dataset):
    def __init__(self, data_dir, mode, args, transform, truncate_testset=False):
        """
        Args:
            data_dir (str): Path to the directory containing the text files.
            mode (str): 'train' or 'test', determines which data to load.
            args (dict): Program arguments, including 'act_mode_8bit'.
            truncate_testset (bool): Whether to truncate the test set (default: False).
        """
        self.data_dir = data_dir
        self.mode = mode
        self.args = args
        self.transform = transform
        self.truncate_testset = truncate_testset

        # Load the data
        self.file_paths = [os.path.join(data_dir, file) for file in os.listdir(data_dir) if file.endswith('.json')]
        self.data = []
        self.labels = []

        # Read each file, load matrices, and assign labels
        for idx, file_path in enumerate(self.file_paths):
            with open(file_path, 'r') as file:
                data = json.load(file)
                for matrix_data in data:

                    # Convert to PyTorch tensor of float32
                    tensor_data = torch.tensor(matrix_data, dtype=torch.float32)

                    #print(tensor_data.max())
                
                    # Add tensor data and the corresponding label
                    self.data.append(tensor_data)
                    self.labels.append(idx)

        # Optionally truncate the test set
        if self.mode == 'test' and self.truncate_testset:
            self.data = self.data[:1]
            self.labels = self.labels[:1]

        if self.transform is not None:
            self.data = [self.transform(im).type(torch.float) for im in self.data]

        #print(self.data)

    def __len__(self):
        """Returns the size of the dataset"""
        return len(self.data)

    def __getitem__(self, idx):
        """Returns the ith data sample and its corresponding label"""
        data_sample = self.data[idx]
        label = self.labels[idx]
        
        return data_sample, label

def imu_get_datasets(data, load_train=True, load_test=True):
    """
    Load the custom dataset where each text file contains a semicolon-separated matrix.
    
    Args:
        data (tuple): Contains the data directory and program arguments.
        load_train (bool): Whether to load the training data (default: True).
        load_test (bool): Whether to load the testing data (default: True).
    
    Returns:
        train_dataset, test_dataset: Loaded datasets for training and testing.
    """
    data_dir, args = data

    transform = transforms.Compose([
        ai8x.normalize(args=args),
    ])

    # Load training dataset if requested
    if load_train:
        train_dataset = IMU_AI(data_dir=data_dir, mode='train', args=args, transform=transform, truncate_testset=False)
    else:
        train_dataset = None

    # Load testing dataset if requested
    if load_test:
        test_dataset = IMU_AI(data_dir=data_dir, mode='test', args=args, transform=transform, truncate_testset=False)
    else:
        test_dataset = None

    return train_dataset, test_dataset

datasets = [
    {
        'name': 'IMU_AI',
        'input': (30, 6, 6),
        'output': (0,1,2,3,4),
        'loader': imu_get_datasets
    },
]

