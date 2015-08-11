#!/usr/bin/env python
import sys, os
sys.path.append(os.path.realpath('pi/cv'))
import flask
from flask import Flask, Response, request
import requests
from threading import Thread
import time,sys
import Queue
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose, Quaternion, Point
from geometry_msgs.msg import Vector3
from hector_uav_msgs.msg import Altimeter
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry

from std_srvs.srv import Empty
from functools import partial
from gevent import pywsgi
import signal
from wsgiref.simple_server import make_server
import webbrowser
import json
from node_controller import *

class talkerNode:
    def __init__(self, landing_para):
        self.queue = Queue.Queue()

        self.allstatus = ['ready', 'nav', 'landing']
        self.status = 'ready'
        self.controller = None
        self.landing_para = landing_para
        self.nav_para = {}
        self.goal = [0, 0]

        self.ptime = 0.0
        self.battery = 1.0

        self.altimeter = 0
        self.metersub = rospy.Subscriber('/altimeter', Altimeter, self.altimeterCallback)

        self.pos = [0, 0, 0, 0, 0, 0]
        self.possub = rospy.Subscriber('/ground_truth/state', Odometry, self.positionCallback)

        self.camlist = ['station', 'front']
        self.camsub = {}
        self.camdata = {}
        for cam in self.camlist:
            self.camsub[cam] = rospy.Subscriber('/%s_cam/camera/image/compressed' % cam,
                                                CompressedImage, partial(self.camCallback, cam))
            self.camdata[cam] = ''

        self.movepub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        rospy.wait_for_service('/gazebo/reset_world')
        self.resetsrv = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.quiting = False

    def start_landing(self):
        pass
    def positionCallback(self, data):
        res = []
        pos = data.pose.pose.position
        res.extend([pos.x, pos.y, pos.z])
        q = data.pose.pose.orientation
        res.extend(euler_from_quaternion([q.w, q.x, q.y, q.z]))
        self.pos = res
        #print res

    def camCallback(self, cam, data):
        self.camdata[cam] = data.data
        #rospy.loginfo("get cam info from " + cam)

    def altimeterCallback(self, data):
        self.altimeter = data.altitude

    def talk(self):
        self.ptime = time.time()
        while not rospy.is_shutdown() and not self.quiting:
            self.battery -= 0.00005*(time.time() - self.ptime)
            self.battery = 0 if self.battery < 0 else self.battery
            try:
                name, msg = self.queue.get(True, 0.1)
                if self.status == 'ready':
                    if name == 'move':
                        self.movepub.publish(msg)
                    elif name == 'reset':
                        self.resetsrv()
                    elif name == 'landing':
                        node.controller = LandingNodeController(self, self.landing_para)
                        self.status = 'landing'
                    elif name == 'nav':
                        node.controller = NavNodeController(self, self.nav_para)
                        self.status = 'nav'

                elif self.status == 'landing':
                    if name == 'ready':
                        self.stop()
                        self.controller = None
                        self.status = 'ready'

                elif self.status == 'nav':
                    if name == 'ready':
                        self.stop()
                        self.controller = None
                        self.status = 'ready'
                # self.rate.sleep()
                self.queue.task_done()
            except rospy.ServiceException:
                self.queue.task_done()
            except Queue.Empty:
                pass
            if self.status == 'landing':
                m = self.controller.update()
                if m:
                    move = Twist(Vector3(m[0],m[1],m[2]), Vector3(m[3],m[4],m[5]))
                    self.movepub.publish(move)
                else:
                    self.stop()
                    self.battery = 1.0
                    self.controller = None
                    self.status = 'ready'
            elif self.status == 'nav':
                m = self.controller.update()
                if m:
                    move = Twist(Vector3(m[0],m[1],m[2]), Vector3(m[3],m[4],m[5]))
                    self.movepub.publish(move)
                else:
                    self.stop()
                    self.controller = None
                    self.status = 'ready'

    def update(self, name, msg):
        self.queue.put((name, msg))

    def stop(self):
        self.movepub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

    def quit(self):
        self.quiting = True

def gps2pos(gps):
    return [1000*(gps[0]-113.659261), 1000*(gps[1]-34.799606)]

def pos2gps(pos):
    return [113.659261 + pos[0]*0.001, 34.799606 + pos[1]*0.001]

app = Flask(__name__,static_url_path='/static')
app.debug = True

@app.route('/')
def index():
    return app.send_static_file('control.html')

@app.route('/data/altimeter')
def altimeter():
    return str(node.altimeter)

@app.route('/action/set_status', methods=['POST'])
def set_status():
    status = request.form['status']
    if status not in node.allstatus:
        return '-1'
    node.update(status, None)
    return '0'

@app.route('/action/set_goal', methods=['POST'])
def set_goal():
    gx, gy = float(request.form['gx']), float(request.form['gy'])
    node.goal = gps2pos([gx, gy])
    print gx, gy
    return '0'

@app.route('/data/all')
def alldata():
    x = 113.659261 + node.pos[0]*0.001
    y = 34.799606 + node.pos[1]*0.001
    data = {
        'status': node.status,
        'altimeter': '12.3',
        'gps': pos2gps([node.pos[0], node.pos[1]]),
        'pose': node.pos[3],
        'battery': node.battery,
    }
    return json.dumps(data)

@app.route('/data/position')
def position():
    return ','.join(map(str, node.pos))

@app.route('/data/cam/<cam>')
def cam(cam):
    if cam not in node.camlist: return '-1'
    return Response(node.camdata[cam], mimetype='image/jpeg')

@app.route('/action/reset')
def reset():
    node.update('reset', None)
    return '0'

@app.route('/action/moven', methods=['POST'])
def moven():
    p = {'x':0, 'y':0, 'z':0,
         'a':0, 'b':0, 'c':0}
    for m in p:
        p[m] = float(request.form[m])
    node.update('move',
        Twist(Vector3(p['x'],p['y'],p['z']),
              Vector3(p['a'],p['b'],p['c'])))
    return '0'

@app.route('/move/<m>')
def move(m):
    movements = {
        "stop" : Twist(Vector3(0,0,0),Vector3(0,0,0)),
        "forward" : Twist(Vector3(1,0,0),Vector3(0,0,0)),
        "backward" : Twist(Vector3(-1,0,0),Vector3(0,0,0)),
        "left" : Twist(Vector3(0,1,0),Vector3(0,0,0)),
        "right" : Twist(Vector3(0,-1,0),Vector3(0,0,0)),
        "up" : Twist(Vector3(0,0,1),Vector3(0,0,0)),
        "down" : Twist(Vector3(0,0,-1),Vector3(0,0,0)),
        "leftt" : Twist(Vector3(0,0,0),Vector3(0,0,1)),
        "rightt" : Twist(Vector3(0,0,0),Vector3(0,0,-1)),
    }
    if m not in movements:
        return "-1"
    node.update('move', movements[m])
    return "0"




def httpserver():
    global wserver
    rospy.loginfo('Server init.')
    wserver = pywsgi.WSGIServer(('', 5000), app)
    wserver.serve_forever()
    #app.run(debug=True)

landing_para = {'marks': ['r', 'g', 'b'],
            'r': [0, 10],
            'g': [60, 10],
            'b': [120, 10],
            'img_h': 240,
            'img_w': 320,
            'height_coeff': 18.7,
            'x_coeff': 1,
            'y_coeff': 1,
            'c_pid': {'g': 0, 'p': 0.5, 'i': 0.0, 'd': 0.0, 'n': -1},
            'xy_pid': {'g': 0, 'p': 0.4, 'i': 0.0, 'd': 0.0, 'n': -1},
            'y_pid': {'g': 0, 'p': 0.4, 'i': 0.0, 'd': 0.0, 'n': -1},
            'z_pid': {'g': 0.5, 'p': 0.6, 'i': 0.0, 'd': 0.0, 'n': -1}
            }
if __name__ == '__main__':
    global node
    node = talkerNode(landing_para)
    def stop_server(*args, **kwargs):
        os.kill(os.getpid(), 9)
    signal.signal(signal.SIGINT,  stop_server)
    server = Thread(target=httpserver)
    server.damon = True
    server.start()
    #webbrowser.open('http://127.0.0.1:5000')
    #app.run(host='0.0.0.0', port=5000, debug= True)
    #worker = Thread(target=node.talk);worker.start()
    #wserver.serve_forever()
    node.talk()
