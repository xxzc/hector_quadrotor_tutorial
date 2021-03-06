#!/usr/bin/env python
import flask
from flask import Flask, Response, request
import time,sys,math
from gevent import pywsgi
from wsgiref.simple_server import make_server
import os,json
def relpath(fn):
    return os.path.join(os.path.dirname(__file__), fn)

class talkerNode:
    def __init__(self, topic):
        self.camlist = ['station', 'front']
        self.camdata = {}
        self.status = 'ready'
        for cam in self.camlist:
            with open(relpath('static/' + cam + '.jpg')) as f:
                self.camdata[cam] = f.read()

app = Flask(__name__,static_url_path='/static')
app.debug = True

@app.route('/')
def index():
    return app.send_static_file('control.html')

@app.route('/data/altimeter')
def altimeter():
    return '12.3'

@app.route('/data/position')
def position():
    return '0,0,0,0,0,0'

@app.route('/data/gps')
def gps():
    x = 113.659261 + math.cos(time.time())*0.001
    y = 34.799606 + math.sin(time.time())*0.001
    return '[%f, %f]' % (x, y)

@app.route('/data/all')
def alldata():
    x = 113.659261 + math.cos(time.time())*0.001
    y = 34.799606 + math.sin(time.time())*0.001
    data = {
        'status': node.status,
        'altimeter': '12.3',
        'gps': [x, y],
    }
    return json.dumps(data)

@app.route('/data/cam/<cam>')
def cam(cam):
    if cam not in node.camlist: return '-1'
    return Response(node.camdata[cam], mimetype='image/jpeg')

@app.route('/action/reset')
def reset():
    return '0'

@app.route('/action/set_status', methods=['POST'])
def set_status():
    node.status = request.form['status']
    return '0'

@app.route('/action/moven', methods=['POST'])
def moven():
    return '0'

@app.route('/move/<m>')
def move(m):
    return "0"

def httpserver():
    global wserver

    #app.run(debug=True)

if __name__ == '__main__':
    global node
    node = talkerNode('cmd_vel')
    wserver = pywsgi.WSGIServer(('', 5000), app)
    print 'server init.'
    wserver.serve_forever()
