#!/usr/bin/env python
import flask
from flask import Flask, Response
import sys
from threading import Thread

camdata = dict()

def reader():
	while True:
		cmd = 'raspistill -t 100 -w 100 -h 100 -o img.jpg' #wabout -t
		with open('img.jpg') as f:
			camdata['station'] = f.read()
			
app = Flask(__name__,static_url_path='/static')
@app.route('/')
def index():
    return app.send_static_file('control.html')

@app.route('/data/cam/<cam>')
def cam(cam):
    if cam not in node.camlist: return '-1'
    return Response(camdata[cam], mimetype='image/jpeg')

if __name__ == '__main__':
	r = Thread(target=reader)
	r.start()
	app.run(host='0.0.0.0', port=5000, debug=False)