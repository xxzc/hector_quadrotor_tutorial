#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import flask
from flask import Flask, Response, request
from threading import Thread
from gevent import pywsgi
import signal
from wsgiref.simple_server import make_server
import code, traceback, signal, threading, sys

def debug(signal, frame):
    id2name = dict([(th.ident, th.name) for th in threading.enumerate()])
    code = []
    for threadId, stack in sys._current_frames().items():
        code.append("\n# Thread: %s(%d)" % (id2name.get(threadId,""), threadId))
        for filename, lineno, name, line in traceback.extract_stack(stack):
            code.append('File: "%s", line %d, in %s' % (filename, lineno, name))
            if line:
                code.append("  %s" % (line.strip()))
    print "\n".join(code)
    with open('st.txt', 'w') as f:
        f.write("\n".join(code))
    wserver.server_close()

def listen():
    signal.signal(signal.SIGINT, debug)  # Register handler

app = Flask(__name__)
app.debug = True

@app.route('/')
def index():
    return 'Works!'

def httpserver():
    global wserver
    rospy.loginfo('Server init.')
    wserver = pywsgi.WSGIServer(('', 5000), app)
    wserver.serve_forever()

def nodeth():
    global node
    node = Node()
    node.talk()

class Node:
    def talk(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()

if __name__ == '__main__':
    listen()
    global node
    node = Node()
    server = Thread(target=httpserver)
    server.damon = True
    server.start()
    node.talk()
