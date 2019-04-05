import cv2
import numpy as np
import os
import argparse

# parser = argparse.ArgumentParser(description='arguments')
# parser.add_argument('--target', action='store_true', help='plot paths')
# args = parser.parse_args()

from os.path import isfile, join

# Here is all the parameters you need to change
fps = 5.0 # shuould be 1/dt where dt is your sampling rate
target = 'radar' # should be the type of figure you want to plot
                 # either: lidar, radar, or dynamic
file_num = 87 # the total number of samples you have.
              # should be the largest number in your lidar_step#.dat
# End of all the parameters definition

frame_array = []

for i in range(file_num):
    filename='output_img/' + target + str(i+1) + '.png'
    #reading each files
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width,height)
    print(filename)
    #inserting the frames into an image array
    frame_array.append(img)

out = cv2.VideoWriter('video/' + target + '.avi',cv2.cv.CV_FOURCC(*'XVID'), fps, size)

for i in range(len(frame_array)):
    # writing to a image array
    out.write(frame_array[i])
out.release()
