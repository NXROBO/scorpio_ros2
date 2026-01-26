#!/bin/bash


gnome-terminal --title="scorpio_ontrol" --geometry 34x10+63+305 -- bash -c "ros2 run scorpio_teleop scorpio_teleop_node 0.14 0.5"

