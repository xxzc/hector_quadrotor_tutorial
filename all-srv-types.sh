#!/bin/bash

rosservice list
echo

for f in `rosservice list`
do
    echo ==$f==
    t=`rosservice type $f`
    echo $t
    rossrv show $t
done
