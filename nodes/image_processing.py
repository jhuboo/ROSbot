#!/usr/bin/env python

"""This is a library of functions for performing color-based image segmentation of an image."""

import cv2
import numpy as np

def img_patch(img,box):
    """ Returns a region of interest of img specified by box """
    #check box against the boundaries of the image
    if box[0]<0:
        box[0]=0
    if box[1]<0:
        box[1]=0
    if box[2]>img.shape[0]:
        box[2]=img.shape[0]
    if box[3]>img.shape[1]:
        box[3]=img.shape[1]
    
    return img[box[0]:box[2],box[1]:box[3],:]

def img_patch_show(img,box,window_name):
    """ Show a region of interest of img specified by box """
    region=img_patch(img,box)
    cv2.imshow(window_name,region)

def pixel_classify(p):
    """ Classify a pixel as background or foreground accoriding to a set of predefined rules """
    #This implementation is a stub. You should implement your own rules here.
    if sum(p)<20:
        return -1.0
    else:
        return 1.0
    
def img_classify(img):
    """ Classify each pixel in an image, and create a black-and-white mask """
    img_segmented=img.copy()
    for r in xrange(0,img.shape[0]):
        for c in xrange(0,img.shape[1]):
            p=img[r,c,:]
            if pixel_classify(p)<0:
                img_segmented[r,c,:]=0
            else:
                img_segmented[r,c,:]=255
    return img_segmented

def img_line_vertical(img,x):
    """ Adds a green 3px vertical line to the image """
    img_line=img.copy()
    cv2.line(img_line, (x, 0), (x, img.shape[1]), (0,255,0), 3)
    return img_line
    
if __name__ == '__main__':
    #load sample image
    img=cv2.imread('../data/BU_logo.png',cv2.IMREAD_COLOR)
    #show sample region
    img_patch_show(img,[50,20,70,40],'region')
    #run classifier to segment image
    img_segmented=img_classify(img)
    #add a line at 10px from the left edge
    img_segmented_line=img_line_vertical(img,10)
    #show results
    cv2.imshow('segmented',img_segmented_line)
    cv2.waitKey(5000)
    cv2.destroyAllWindows()
    


