#!/usr/bin/env python
import flask
from flask import Flask, Response
import requests
from threading import Thread
import time,sys 
import Queue
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from hector_uav_msgs.msg import Altimeter
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Empty
from functools import partial


class talkerNode:
    def __init__(self, topic):
        self.queue = Queue.Queue()
        
        self.altimeter = 0
        self.metersub = rospy.Subscriber('/altimeter', Altimeter, self.altimeterCallback)
        
        self.camlist = ['station', 'front']
        self.camsub = {}
        self.camdata = {}
        for cam in self.camlist:
            self.camsub[cam] = rospy.Subscriber('/%s_cam/camera/image/compressed' % cam, 
                        CompressedImage, partial(self.camCallback, cam))
            self.camdata[cam] = ''
            
        self.movepub = rospy.Publisher(topic, Twist, queue_size=10)
        
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.resetsrv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.quiting = False
        
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


@app.route('/')
def index():
    return app.send_static_file('control.html')

@app.route('/data/altimeter')
def altimeter():
    return str(node.altimeter)

@app.route('/data/cam/<cam>')
def cam(cam):
    if cam not in node.camlist: return '-1'
    return Response(node.camdata[cam], mimetype='image/jpeg')
    
@app.route('/action/reset')
def reset():
    node.update('reset', None)
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
    
@app.route('/shutdown', methods=['POST'])
def shutdown():
    func = flask.request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()
    return 'Server shutting down...'

def shutdown_server():
    requests.post('http://localhost:5000/shutdown')
   
    

if __name__ == '__main__':
    try:
        global node
        node = talkerNode('cmd_vel')
        server = Thread(target=app.run, 
                        kwargs={'host': '0.0.0.0', 'port': 5000,'debug': False})
        server.damon = True
        server.start()
        #app.run(host='0.0.0.0', port=5000, debug= True)
        #worker = Thread(target=node.talk);worker.start()
        node.talk()
        rospy.loginfo('Server init.')
        
        #shutdown_server()
    except KeyboardInterrupt:
        print 'C-c'
