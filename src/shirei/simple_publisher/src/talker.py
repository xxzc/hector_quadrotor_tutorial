#!/usr/bin/env python
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
import os
import webbrowser

class talkerNode:
    def __init__(self, topic):
        self.queue = Queue.Queue()

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

        self.movepub = rospy.Publisher(topic, Twist, queue_size=10)

        rospy.wait_for_service('/gazebo/reset_world')
        self.resetsrv = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.quiting = False

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
        while not rospy.is_shutdown() and not self.quiting:
            try:
                name, msg = self.queue.get(True, 1)
                if name == 'move':
                    self.movepub.publish(msg)
                elif name == 'reset':
                    self.resetsrv()
                #self.rate.sleep()
                self.queue.task_done()
            except rospy.ServiceException:
                self.queue.task_done()
            except Queue.Empty:
                pass


    def update(self, name, msg):
        self.queue.put((name, msg))

    def quit(self):
        self.quiting = True

app = Flask(__name__,static_url_path='/static')
app.debug = True

@app.route('/')
def index():
    return app.send_static_file('control.html')

@app.route('/data/altimeter')
def altimeter():
    return str(node.altimeter)

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

if __name__ == '__main__':
    global node
    node = talkerNode('cmd_vel')
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
