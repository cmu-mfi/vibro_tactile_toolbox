#!/usr/bin/env python3

'''
Custom torch class for loading VibroTactile Toolbox datasets from create_training_dataset.py
'''

import torch
from torch.utils.data import Dataset

from typing import List

from argparse import ArgumentParser

VOLUMES = ['25', '50', '75', '100']
BRICK_TYPES = ['2x1', '2x2', '2x4', '4x2', '4x1']
VELOCITIES = ['0.01', '0.02', '0.05', 'rand']
LABELLING_OPTIONS = ['none',                # do not produce labels
                     'outcome_detector',    # label based on outcome detections
                     'policy_correction']   # label based on pose error
SKILL_OPTIONS = ['MoveDown',
                 'PickLego',
                 'PlaceLego']

class VTTDataset(Dataset):
    def __init__(self, 
                 data_path: str, 
                 skill: str, 
                 volume: List[str], 
                 brick_type: List[str], 
                 velocity: List[str], 
                 labelling: str):
        """
        Args:
            data_path (string): Path to the data directory.
        """
        print(f"Initializing VTTDataset")

        self.src = data_path
        self.skill = skill
        self.volumes = volume
        self.brick_types = brick_type
        self.velocities = velocity
        self.labelling_method = labelling

        print(f"data_path: {data_path}")
        print(f"skill: {skill}")
        print(f"volume: {volume}")
        print(f"brick_type: {brick_type}")
        print(f"velocity: {velocity}")
        print(f"labelling: {labelling}")

        self.data = self.load_data(data_path)
        self.labels = self.load_labels(data_path)
    
    def load_data(self, data_path: str):
        """
        Loads data from the data path.
        data_path/
            {volume}/
                {brick_type}/
                    {velocity}/
                        trial_{n}_p_{perturbation}_{velocity}_{skill}_{outcome}

        
        Args:
            data_path (string): Path to the data directory.
            
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
    
    def __str__(self):
        dataset_info = super().__str__()
        dataset_info += '\n'
        dataset_info += 'Bingus'
        return dataset_info

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
    parser = ArgumentParser()
    parser.add_argument('--src', '-s', type=str, required=True,
                        help='Src of dataset to load.')
    parser.add_argument('--skill', type=str, required=True,
                        choices=SKILL_OPTIONS,
                        help="Skill to select training examples from. Select from {'MoveDown', 'PickLego', 'PlaceLego'}.")
    parser.add_argument('--volume', nargs='+', type=str, default=['all'],
                        choices=(VOLUMES + ['all']),
                        help="Volume levels to include in dataset. Select from {'25', '50', '75', '100'} or 'all'.")
    parser.add_argument('--brick_type', nargs='+', type=str, default=['all'],
                        choices=(BRICK_TYPES + ['all']),
                        help="Brick types to include in dataset. Select from {'2x1', '2x2', '2x4', '4x2', '4x1'} or 'all'.")
    parser.add_argument('--velocity', nargs='+', type=str, default=['all'],
                        choices=(VELOCITIES + ['all']),
                        help="MoveDown velocity scaling levels to include in dataset. Select from {'0.01', '0.02', '0.05', 'rand'} or 'all'.")
    parser.add_argument('--labelling', type=str, default='outcome_detector',
                        choices=LABELLING_OPTIONS,
                        help="Datset labelling strategy. Select from {'None', 'outcome_detector', 'policy_correction'}.")
    args = parser.parse_args()

    if 'all' in args.volume:
        args.volume = VOLUMES
    else:
        # remove duplicates
        args.volume = list(set(args.volume))

    if 'all' in args.brick_type:
        args.brick_type = BRICK_TYPES
    else:
        # remove duplicates
        args.brick_type = list(set(args.brick_type))

    if 'all' in args.velocity:
        args.velocity = VELOCITIES
    else:
        args.velocity = list(set(args.velocity))

    dataset = VTTDataset(
        data_path=args.src, 
        skill=args.skill,
        volume=args.volume, 
        brick_type=args.brick_type, 
        velocity=args.velocity, 
        labelling=args.labelling
    )
    
    print(dataset)
