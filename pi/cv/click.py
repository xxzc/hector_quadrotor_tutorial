#!/usr/bin/python
import os
from sys import argv

relx = [920,1050]
rely = [620,500]
rerx = [1170,1300]
rery= rely

def rmap(ch, n):
    return ch[0]+ (ch[1]-ch[0])*(n+1)/2

def mmove(x, y):
    os.system('xdotool mousemove %d %d' % (x, y))
    
def mclick():
    os.system('xdotool click 1')

def main(ch1, ch2, ch3, ch4):
    mmove(rmap(relx, ch2),rmap(rely, ch1))
    mclick()
    mmove(rmap(rerx, ch3),rmap(rery, ch4))
    mclick()

if __name__ ==  '__main__':
    main(*map(int,[argv[1],argv[2],argv[3],argv[4]]))
