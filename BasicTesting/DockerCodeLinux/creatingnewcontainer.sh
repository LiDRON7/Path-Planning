#!/bin/bash
xhost +local:docker
docker run -it \
  --net=host \
  --name "px4dev" \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri \
  --memory="16g" \
  px4-gazebo

