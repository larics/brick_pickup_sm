#!/bin/bash
roslaunch brick_pickup_sm brick_spawn.launch color:=Red number:=0 x:=3 y:=-4
sleep 5
roslaunch brick_pickup_sm brick_spawn.launch color:=Red number:=1 x:=3.5 y:=-4
sleep 5
roslaunch brick_pickup_sm brick_spawn.launch color:=Red number:=2 x:=4 y:=-4
sleep 5
roslaunch brick_pickup_sm brick_spawn.launch color:=Green number:=3 x:=3 y:=-5
sleep 5
roslaunch brick_pickup_sm brick_spawn.launch color:=Green number:=4 x:=3.5 y:=-5