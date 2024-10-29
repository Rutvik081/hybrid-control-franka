#!/bin/bash
set -e
set -u

xhost +

docker run -it \
--runtime=nvidia \
--privileged \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/embodied/Desktop/frankapy/examples:/opt/frankapy/examples:rw \
-v /home/embodied/Desktop/frankapy/frankapy:/opt/frankapy/frankapy:rw \
-v /dev/serial/by-id/:/dev/serial/by-id/ \
--device=/dev/video0:/dev/video0 \
-e DISPLAY=$DISPLAY \
--network=host \
--name=frankapy_container frankapy
xhost -