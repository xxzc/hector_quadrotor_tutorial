#!/bin/bash
if [ $# = 1 ]; then
  python src/shirei/simple_publisher/src/talker_dum.py
else
  rosrun simple_publisher talker.py
fi
