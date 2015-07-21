#!/usr/bin/env python
import flask
from flask import Flask, Response
import sys

img = ''

def reader():
	while true:
		cmd = 'raspistill -t 100 -w 100 -h 100 -o img.jpg'
		with open('img.jpg') as f:
			img = f.read()