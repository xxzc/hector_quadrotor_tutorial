# -*- coding: utf-8 -*-
from time import clock
import urllib
from geo import angle
import numpy as np
import math

def move_req(x, y, z, a, b, c):
    print x, y, z, a, b, c
    url = 'http://127.0.0.1:5000/action/moven'
    para = urllib.urlencode({'x': x, 'y': y, 'z': z,
                             'a': a, 'b': b, 'c': c})
    return urllib.urlopen(url, para).read()


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