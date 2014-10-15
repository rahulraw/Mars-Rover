#!/bin/bash

roscore&
rosrun cameras camera.py $1 $2 $3 $4&
rosrun cameras receiver.py $3 $5
