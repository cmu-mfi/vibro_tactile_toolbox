#!/usr/bin/env python3

'''
Custom torch class for loading VibroTactile Toolbox datasets from create_training_dataset.py
'''

import torch
from torch.utils.data import Dataset

class VTTDataset(Dataset):
    def __init__(self, data_path):
        """
        Args:
            data_path (string): Path to the data file or directory.
        """
        # Load data from the data_path
        self.data = self.load_data(data_path)
        # If you have labels
        self.labels = self.load_labels(data_path)
    
    def load_data(self, data_path):
        """
        Loads data from the data path.
        Implement this method based on your data format.
        
        Args:
            data_path (string): Path to the data file or directory.
            
        Returns:
            list: A list or another data structure containing your data.
        """
        # Implement your data loading logic here
        data = []
        # Example:
        # with open(data_path, 'r') as file:
        #     data = file.readlines()
        return data
    
    def load_labels(self, data_path):
        """
        Loads labels from the data path if you have labels.
        Implement this method based on your data format.
        
        Args:
            data_path (string): Path to the data file or directory.
            
        Returns:
            list: A list or another data structure containing your labels.
        """
        # Implement your label loading logic here
        labels = []
        # Example:
        # with open(data_path, 'r') as file:
        #     labels = file.readlines()
        return labels

    def __len__(self):
        """
        Returns the total number of samples in the dataset.
        
        Returns:
            int: The total number of samples.
        """
        return len(self.data)
    
    def __getitem__(self, idx):
        """
        Generates one sample of data.
        
        Args:
            idx (int): Index of the sample to retrieve.
        
        Returns:
            tuple: (sample, label) where label is optional.
        """
        sample = self.data[idx]
        # If you have labels, return (sample, label)
        label = self.labels[idx]
        return sample, label
        # If you don't have labels, return sample
        # return sample

# Example usage
if __name__ == "__main__":
    dataset = CustomDataset(data_path="path/to/your/data")
    print("Number of samples:", len(dataset))
    for i in range(len(dataset)):
        sample, label = dataset[i]
        print(f"Sample #{i}: {sample}, Label: {label}")
