#!/bin/bash


gnome-terminal --tab -x bash -c "roslaunch px4 robocup2025.launch" &

gnome-terminal --tab -x bash -c "sleep 5;roslaunch px4 spawn_single_typhoon_0.launch" & 
gnome-terminal --tab -x bash -c "sleep 10;roslaunch px4 spawn_single_typhoon_1.launch" & 
gnome-terminal --tab -x bash -c "sleep 15;roslaunch px4 spawn_single_typhoon_2.launch" & 
gnome-terminal --tab -x bash -c "sleep 20;roslaunch px4 spawn_single_typhoon_3.launch" & 
gnome-terminal --tab -x bash -c "sleep 25;roslaunch px4 spawn_single_typhoon_4.launch" & 
gnome-terminal --tab -x bash -c "sleep 30;roslaunch px4 spawn_single_typhoon_5.launch" 



