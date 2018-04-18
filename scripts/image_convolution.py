#!/usr/bin/env python
import cv2
import numpy as np

img=cv2.imread('BU_logo.png',cv2.IMREAD_COLOR) #by default, imread reads the image in BGR format

#change to grayscale
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#show dimensions of the image
print img.shape

kernel_vertical=np.array([[-1],[1]]);
kernel_horizontal=np.array([[-1,1]]);

imgVerticalFilter = cv2.filter2D(img_gray,-1,kernel_vertical)
imgHorizontalFilter = cv2.filter2D(img_gray,-1,kernel_horizontal)


#show the original image
cv2.imshow('image',img) 

#show the processed image
cv2.imshow('Vertical filter', imgVerticalFilter)
cv2.imshow('Horizontal filter', imgHorizontalFilter)

cv2.waitKey(5000) # wait for a  key or 5000ms (5s)
cv2.destroyAllWindows() #close all windows
