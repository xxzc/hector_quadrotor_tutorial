#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
from control import *

def no(val):
    pass

if __name__ ==  '__main__':
    #cap = cv2.VideoCapture(0)
    #img = cv2.imread('station.jpg')
    cam = initcam()
    img = getimg(cam)
    #ret, img = cap.read()
    h, w, _ = img.shape
    color = create_image(h, w)
    hsv =  cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.namedWindow('image')
    cv2.createTrackbar('C','image',0,180, no)
    cv2.createTrackbar('th','image',0,30, no)
    while(True):
        img = getimg(cam)
        hsv =  cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        c = cv2.getTrackbarPos('C','image')
        threshold = cv2.getTrackbarPos('th','image')
        color[:,:] = hsv2bgr((c,255,255))
        mask = hsv_inrange(hsv, c, threshold, 50, 50)
        mask = cv2.bitwise_and(color,color,  mask=mask)
        cv2.imshow('image',mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    #cap.release()   
    cv2.destroyAllWindows()
