#!/usr/bin/python
import numpy as np
import cv2
import urllib
from control import *
from geo import *


def url_to_image(url):
    resp = urllib.urlopen(url)
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    return image


def create_image(h, w):
    return np.zeros((h, w, 3), np.uint8)


def hsv2bgr(hsv):
    return cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()


def hsv_inrange(hsv, c, th, slo, vlo):
    lo = (c - th) % 180
    hi = (c + th) % 180
    if lo <= hi:
        low = np.array([lo, slo, vlo])
        high = np.array([hi, 255, 255])
        return cv2.inRange(hsv, low, high)
    else:
        p1 = cv2.inRange(hsv, np.array([lo, slo, vlo]),
                         np.array([179, 255, 255]))
        p2 = cv2.inRange(hsv, np.array([0, slo, vlo]),
                         np.array([hi, 255, 255]))

        return p1 + p2


def get_block(hsv, color, threshold):
    mask = hsv_inrange(hsv, color, threshold, 50, 50)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    circles = map(cv2.minEnclosingCircle, contours)
    if circles:
        circle, contour = max(zip(circles,contours), key=lambda b: b[0][1])
        contour = cv2.approxPolyDP(contour,0.005*cv2.arcLength(contour,True), True)
        ccenter = (sum(map(lambda c:c[0][0], contour))/len(contour),
                             sum(map(lambda c:c[0][1], contour))/len(contour))
    else:
        circle, contour,ccenter = None, None, None
    return circle, contour, ccenter, mask


def odometry(image, para):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    res = {}
    for p in para['marks']:
        color, threshold = para[p]
        circle, contour, ccenter, mask = get_block(hsv, color, threshold)
        if not circle:
            return
        res[p + '_c'] = (int(circle[0][0]),int(circle[0][1]))
        res[p + '_r'] = circle[1]
        res[p + '_ct'] = contour
        res[p + '_cc'] = ccenter
    return res


def paint_odometry(image, data, para, extra=True):
    if extra:
        for p in para['marks']:
            if p+'_c' in data:
                cv2.circle(image, data[p + '_c'], int(data[p + '_r']),
                           hsv2bgr((para[p][0], 255, 255)), 2)
                cv2.drawContours(image, [data[p+'_ct']], -1, (0,255,0))
                cv2.line(image, data[p + '_c'], data[p + '_cc'], (255, 255, 0), 2)
    return image


def getimg():
    return cap.read()[1]
    # return cv2.imread('station.jpg')
    #return url_to_image('http://127.0.0.1:5000/data/cam/station')


if __name__ == '__main__':
    global cap
    cap = cv2.VideoCapture(0)
    frame = getimg()
    odo_para = {'marks': ['r'],
                    'r': [170, 10],
                    #'g': [60, 10],
                    #'b': [120, 10],
                    'img_h': frame.shape[0],
                    'img_w': frame.shape[1],
                    'height_coeff': 18.7,
                    'x_coeff': 1,
                    'y_coeff': 1,
                    'c_pid': {'g': 0, 'p': 0.5, 'i': 0.0, 'd': 0.0, 'n': -1},
                    'xy_pid': {'g': 0, 'p': 0.4, 'i': 0.0, 'd': 0.0, 'n': -1},
                    'y_pid': {'g': 0, 'p': 0.4, 'i': 0.0, 'd': 0.0, 'n': -1},
                    'z_pid': {'g': 0.5, 'p': 0.6, 'i': 0.0, 'd': 0.0, 'n': -1}
                }
    while True:
        # Capture frame-by-frame
        frame = getimg()

        # para = [[0,5],
        #                       [50, 9],
        #                       [104, 10]
        #                       ]

        odo_data = odometry(frame, odo_para)
        if odo_data:
            paint_odometry(frame, odo_data, odo_para)

        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # When everything done, release the capture
        # cap.release()
    cv2.destroyAllWindows()
