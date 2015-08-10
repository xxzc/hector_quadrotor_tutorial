from control import *
from landing import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from math import *

class LandingNodeController:

    def __init__(self, node, para):
        self.para = para
        self.node = node
        self.control = Control(para)

    def update(self):
        frame = string_to_image(self.node.camdata['station'])
        odo_data, state = odometry(frame, self.para)
        if odo_data:
            move = self.control.update(state)
            return move
        else:
            return None

class NavNodeController:

    def __init__(self, node, para):
        self.para = para
        self.node = node

    def update(self):
        x, y, z, r = self.node.pos[0], self.node.pos[1], self.node.pos[2], self.node.pos[3]
        gx, gy = self.node.goal
        # print self.node.pos
        r = pi - r
        tth = pi/2 - atan2(gx-x, gy-y)
        d = abs(gx-x) + abs(gy-y)
        if d < 0.1:
            return None
        n = -1/(d+1) + 1
        return (n*cos(tth-r), n*sin(tth-r), 0, 0, 0, 0)
