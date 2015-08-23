#!/usr/bin/python
import numpy as np
import cv2
import urllib
from control import *
from geo import *




def get_block(hsv, color, threshold):
    mask = hsv_inrange(hsv, color, threshold, 50, 50)
    contours, hierarchy  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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


def paint_odometry(image, data, state, para, extra=True):
    if extra:
        for p in para['marks']:
            cv2.circle(image, np2int(data[p + '_c']), int(data[p + '_r']),
                       hsv2bgr((para[p][0], 255, 255)), 2)
    x, d = data['center'], data['heading']
    cv2.line(image, np2int(x), np2int(x + 2 * d), (255, 255, 0), 2)

    def ptext(t):
        cv2.putText(image, t, (0, ptext.y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        ptext.y += 20
    ptext.y = 20
    ptext('A: %f' % state.c)
    ptext('Z: %f' % state.z)
    ptext('X: %f' % state.x)
    ptext('Y: %f' % state.y)
    return image




if __name__ == '__main__':
    # cap = cv2.VideoCapture(0)
    cam = initcam()
    frame = getimg(cam)
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
        frame = getimg(cam)

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
