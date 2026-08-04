#!/bin/bash
# Wrapper so roslaunch can find/respawn leo_backend.py, which lives outside
# catkin_ws and has no pkg/type of its own.
exec python3 /home/lab272/TOUT/leo_backend.py
