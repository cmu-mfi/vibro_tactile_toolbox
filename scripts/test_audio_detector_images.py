import cv2
import os
import matplotlib.pyplot as plt
import numpy as np

online_image_path = '/mnt/hdd1/test_'
offline_image_path = '/mnt/hdd1/trial_1-p_0.0017_-0.0009_-8.5886_0.01_success_1725816362_'

x = 108
y = 146
for channel in range(4):
    current_online_image_path = online_image_path + str(channel) + '.png'
    current_offline_image_path = offline_image_path + str(channel) + '.png'
    online_image = cv2.imread(current_online_image_path).astype(int)
    offline_image = cv2.imread(current_offline_image_path).astype(int)

    print(online_image[x,y])
    print(offline_image[x,y])

    diff_image = np.abs(online_image - offline_image)
    print(diff_image[x,y])
    plt.imshow(diff_image)
    plt.show()