#!/bin/bash
docker run --name camera --network host --gpus all -it --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix camera:v1.0
