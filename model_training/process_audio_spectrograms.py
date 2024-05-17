
# Copyright 2024 RobotAlert, Inc., All Rights Reserved
# Proprietary and confidential. Unauthorized copying of this file via any medium is strictly prohibited.
# RobotAlert, Inc. is Kevin Zhang (klz1@andew.cmu.edu), Rohit Sonker (rsonker@andrew.cmu.edu)], and Tianqin Li (tianqinl@andrew.cmu.edu)

import os
import torch
import torchaudio
import matplotlib.pyplot as plt
from torch.utils.data import Dataset, DataLoader
from pathlib import Path
import argparse

def load_audio_files(path: str, label:str):

    dataset = []
    walker = sorted(str(p) for p in Path(path).glob(f'*'+label+'/*.wav'))

    for i, file_path in enumerate(walker):
        path, filename = os.path.split(file_path)

        # Load audio
        waveform, sample_rate = torchaudio.load(file_path)
        dataset.append([waveform, sample_rate, label])

    return dataset

def show_waveform(waveform, sample_rate, label):
    print("Waveform: {}\nSample rate: {}\nLabels: {} \n".format(waveform, sample_rate, label))
    new_sample_rate = 16000

    # Resample applies to a single channel, we resample first channel here
    channel = 0
    waveform_transformed = torchaudio.transforms.Resample(sample_rate, new_sample_rate)(waveform[channel,:].view(1,-1))

    print("Shape of transformed waveform: {}\nSample rate: {}".format(waveform_transformed.size(), new_sample_rate))

    plt.figure()
    plt.plot(waveform_transformed[0,:].numpy())

def show_melspectrogram(waveform,sample_rate):
    mel_spectrogram = torchaudio.transforms.MelSpectrogram(sample_rate)(waveform)
    print("Shape of spectrogram: {}".format(mel_spectrogram.size()))

    plt.figure()
    plt.imshow(mel_spectrogram.log2()[0,:,:].numpy(), cmap='viridis')

def show_mfcc(waveform,sample_rate):
    mfcc_spectrogram = torchaudio.transforms.MFCC(sample_rate= sample_rate)(waveform)
    print("Shape of spectrogram: {}".format(mfcc_spectrogram.size()))

    plt.figure()
    fig1 = plt.gcf()
    plt.imshow(mfcc_spectrogram.log2()[0,:,:].numpy(), cmap='viridis')

    plt.figure()
    plt.plot(mfcc_spectrogram.log2()[0,:,:].numpy())
    plt.draw()

def create_spectrogram_images(trainloader, label_dir):
    #make directory
    directory = f'data/spectrograms/{label_dir}/'
    if(os.path.isdir(directory)):
        print("Data exists for", label_dir)
    else:
        os.makedirs(directory, mode=0o777, exist_ok=True)

        for i, data in enumerate(trainloader):

            waveform = data[0]
            sample_rate = data[1]
            label = data[2]

            # create transformed waveforms
            spectrogram_tensor = torchaudio.transforms.Spectrogram()(waveform)

            fig = plt.figure()
            plt.imsave(f'data/spectrograms/{label_dir}/spec_img{i}.png', spectrogram_tensor[0].log2()[0,:].numpy(), cmap='viridis')

def create_mfcc_images(trainloader, label_dir):
    #make directory
    os.makedirs(f'data/mfcc_spectrograms/{label_dir}/', mode=0o777, exist_ok=True)

    for i, data in enumerate(trainloader):

        waveform = data[0]
        sample_rate = data[1]
        label = data[2]

        mfcc_spectrogram = torchaudio.transforms.MFCC(sample_rate= sample_rate)(waveform)

        plt.figure()
        fig1 = plt.gcf()
        plt.imshow(mfcc_spectrogram[0].log2()[0,:].numpy(), cmap='viridis')
        plt.draw()
        fig1.savefig(f'data/mfcc_spectrograms/{label_dir}/spec_img{i}.png', dpi=100)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Processes audio files from a data folder.')
    parser.add_argument('--data_dir', '-d', type=str, required=True,
                        help='Data directory')
    args = parser.parse_args()

    trainset_success = load_audio_files(args.data_dir, 'success')
    trainset_fail = load_audio_files(args.data_dir, 'fail')
    trainset_error = load_audio_files(args.data_dir, 'error')

    trainloader_success = torch.utils.data.DataLoader(trainset_success, batch_size=1,
                                                shuffle=True, num_workers=0)

    trainloader_fail = torch.utils.data.DataLoader(trainset_fail, batch_size=1,
                                                shuffle=True, num_workers=0)

    trainloader_error = torch.utils.data.DataLoader(trainset_error, batch_size=1,
                                                shuffle=True, num_workers=0)

    create_spectrogram_images(trainloader_success, 'success')
    create_spectrogram_images(trainloader_fail, 'fail')
    create_spectrogram_images(trainloader_error, 'error')