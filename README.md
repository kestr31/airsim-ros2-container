- docker run command

```
docker run -it --rm \
-e DISPLAY=$DISPLAY \
-e QT_NO_MITSHM=1 \
-e simhost=localhost \
-v /tmp/.X11-unix:/tmp/.X11-unix 
--net host \
--gpus all \
--name AirSimConTest \
 kestr31/airsim-ros2:test bash
```
