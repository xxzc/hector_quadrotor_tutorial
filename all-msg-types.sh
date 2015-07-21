#!/bin/bash

for f in `rostopic list`
do
    echo ==$f==
    t=`rostopic type $f`
    echo $t
    rosmsg show $t
done
