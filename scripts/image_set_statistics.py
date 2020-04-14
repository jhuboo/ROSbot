#!/usr/bin/env python

from __future__ import print_function

import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys
from mpl_toolkits.mplot3d import Axes3D #Enable 3-D plots
from argparse import ArgumentParser

def images_load(name_list):
    return [{'img': cv2.imread(f),'name':f} for f in name_list]

def images_info(img_list):
    for idx,d in enumerate(img_list):
        out='\tFile %d: %s, ' % (idx,d['name'])
        if d['img'] is None:
            out+='Error loading the file'
        else:
            out+='%d x %d, %d channels' % d['img'].shape
        print(out)

def images_scatter(img_list):
    bins=np.linspace(0,255,256)-0.5
    data_list=[]
    legend=[]
    for d in img_list:
        points=d['img'].transpose((2,1,0)).reshape(3,-1)
        plt.scatter(points[0],points[1],points[2])
        legend.append(d['name'])
    plt.legend(legend)
    plt.xlabel('Channel 0')
    plt.ylabel('Channel 1')
    #plt.zlabel('Channel 2')

def images_histograms_channel(img_list,channel):
    bins=np.linspace(0,255,256)-0.5
    data_list=[]
    legend=[]
    for d in img_list:
        data_list.append(d['img'][:,:,channel].flatten())
        legend.append(d['name'])
    plt.hist(data_list,bins,density=True)
    plt.legend(legend)
    plt.ylabel('Channel %d' % (channel))

def histogram():
    d=np.array([1,1,2,2,3,3,3,4])
    d2=np.array([1,2,2,3,3,4,4,4])
    bins=np.linspace(0,4,5)-0.5
    plt.hist([d,d2], bins)
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.title('Channel')
    plt.show()

def main(name_list):
    img_list=images_load(name_list)
    images_info(img_list)
    for channel in (0,1,2):
        plt.subplot(3, 1, channel+1);
        images_histograms_channel(img_list,channel)
    #plt.subplot(1,2,2,projection='3d')
    #images_scatter(img_list)
    plt.show()

def test():
    name_list=['train-negative.png','train-positive.png','cross-positive.png','cross-negative.png']
    main(name_list)

if __name__ == '__main__':
    parser = ArgumentParser(description='Visualize the histograms of the pixel counts across the three channels for all the provided image file names')
    parser.add_argument('file_name', nargs='+', help='list of color image files (if multiple files are provided, the histograms are shown side-by-side)')

    args = parser.parse_args()
    main(args.file_name)


