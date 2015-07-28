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
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    circles = map(cv2.minEnclosingCircle, contours)
    circle = max(circles, key=lambda b: b[1]) if circles else None
    return circle, mask


def odometry(image, para):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    res = {}
    for p in para['marks']:
        color, threshold = para[p]
        circle, mask = get_block(hsv, color, threshold)
        if not circle:
            return None, None
        res[p + '_c'] = np.array(circle[0])
        res[p + '_r'] = circle[1]
    res['center'] = (res['r_c'] + res['g_c'] + res['b_c']) / 3
    res['heading'] = res['r_c'] - (res['g_c'] + res['b_c']) / 2
    res['size'] = np.cross(res['b_c']-res['r_c'], res['g_c']-res['r_c'])
    return res, State(res, para)


def paint_odometry(image, data, state, para, extra=False):
    if extra:
        for p in para['marks']:
            cv2.circle(image, np2int(data[p + '_c']), int(data[p + '_r']),
                       hsv2bgr((para[p][0], 255, 255)), 2)
    x, d = data['center'], data['heading']
    cv2.line(image, np2int(x), np2int(x + 2 * d), (255, 255, 0), 2)
    def ptext(t):
        cv2.putText(image, t, (0, ptext.y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        ptext.y +=20
    ptext.y = 20
    ptext('A: %f' % state.c)
    ptext('Z: %f' % state.z)
    ptext('X: %f' % state.x)
    ptext('Y: %f' % state.y)
    return image


def getimg():
    # return cv2.imread('station.jpg')
    return url_to_image('http://127.0.0.1:5000/data/cam/station')


if __name__ == '__main__':
    # cap = cv2.VideoCapture(0)
    frame = getimg()
    odo_para = {'marks': ['r', 'g', 'b'],
                    'r': [0, 10],
                    'g': [60, 10],
                    'b': [120, 10],
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
    control = Control(odo_para)
    while True:
        # Capture frame-by-frame
        frame = getimg()

        # para = [[0,5],
        #                       [50, 9],
        #                       [104, 10]
        #                       ]

        odo_data, state = odometry(frame, odo_para)
        if odo_data:
            paint_odometry(frame, odo_data, state, odo_para)
            move = control.update(state)
            move_req(*move)
        else:
            move_req(0, 0, 0, 0, 0, 0)
        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # When everything done, release the capture
        # cap.release()
    cv2.destroyAllWindows()
