# -*- coding: utf-8 -*-
from time import clock
import urllib
from geo import angle
import numpy as np
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

def url_to_image(url):
    def string_to_image(str):
        image = np.asarray(bytearray(str), dtype="uint8")
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        return image
    resp = urllib.urlopen(url)
    return string_to_image(resp.read())


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

def move_req(x, y, z, a, b, c):
    print x, y, z, a, b, c
    url = 'http://127.0.0.1:5000/action/moven'
    para = urllib.urlencode({'x': x, 'y': y, 'z': z,
                             'a': a, 'b': b, 'c': c})
    return urllib.urlopen(url, para).read()

def initcam():
    camera = PiCamera()
    camera.resolution = (320, 240)
    raw = PiRGBArray(camera)
    return camera, raw

def getimg(cam):
    camera, raw = cam
    for frame in camera.capture_continuous(raw, format="bgr", use_video_port=True):
        img = raw.array
        raw.truncate(0)
        return img
    #camera.capture(raw, format='bgr')
    #img = raw.array
    #raw.truncate(0)   
    #return img
    #return cap.read()[1]
    #return cv2.imread('station.jpg')
    #return url_to_image('http://127.0.0.1:5000/data/cam/station')

class Pid:
    def __init__(self, g, p, i, d, n):
        self.g = g  # goal
        self.p = p
        self.i = i
        self.d = d
        self.n = n
        self.running = False
        self.pt = 0.0  # prev time
        self.acc = 0.0  # accumulate
        self.px = 0.0  # prev x

    def start(self):
        self.running = True
        self.pt = clock()
        self.acc = 0
        self.px = 0

    def update(self, x):
        if not self.running:
            self.start()
            self.px = x
        dt = clock() - self.pt
        e = x - self.g
        self.acc += e * dt
        return (self.p * e + self.i * self.acc + self.d * (x - self.px) / dt) * self.n

class State:
    def __init__(self, observe, para):
        self.x, self.y, self.z, self.c = self.parse(observe, para)

    def parse(self, data, para):
        """
        :param data: Observed data
        :param para:
        :return: x, y, z, c
        """
        ang = angle(data['heading'])
        height = para['height_coeff']/math.sqrt(math.fabs(data['size']))
        x = data['center'][0]/para['img_w']*2-1
        x *= para['x_coeff']*height
        y = data['center'][1]/para['img_h']*2-1
        y *= para['y_coeff']*height
        # height = data['size']
        return x, y, height, ang


class Control:
    def __init__(self, para):
        self.c_pid = Pid(**para['c_pid'])
        self.xy_pid = Pid(**para['xy_pid'])
        self.z_pid = Pid(**para['z_pid'])

    def update(self, state):
        out_c = self.c_pid.update(state.c)
        out_xy = self.xy_pid.update(math.sqrt(state.x*state.x+state.y*state.y))
        out_z = self.z_pid.update(state.z)
        theta = angle((state.x, state.y)) - state.c
        return out_xy*math.cos(theta), out_xy*math.sin(theta), out_z, 0, 0, out_c
