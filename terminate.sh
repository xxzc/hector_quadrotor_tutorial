#!/bin/bash
echo roslaunch rosmaster rosout gzserver gzclient | xargs -n 1 pkill -9 
