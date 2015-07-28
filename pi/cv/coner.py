#!/usr/bin/python
import numpy as np
import cv2

if __name__ ==  '__main__':
    cap = cv2.VideoCapture(0)

    while(True):
        # Capture frame-by-frame
        ret, img = cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        sift = cv2.SIFT()
        kp = sift.detect(gray,None)

        img=cv2.drawKeypoints(gray,kp)
        # Display the resulting frame
        cv2.imshow('frame',img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
