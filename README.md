## Introduction
This project implements an AHRS (Attitude and Heading Reference System) based on an Error-State Kalman Filter (ESKF).

The filter supports both:

- **6DoF IMU fusion** using gyroscope + accelerometer
- **9DoF IMU fusion** using gyroscope + accelerometer + magnetometer

and can be evaluated in two different input modes:

- **Online streaming mode** from DepthAI IMU sensor
- **Offline replay mode** from recorded WitMotion CSV logs

## Docker setup
1. Building image
    ```shell=
    sudo docker build --network=host -t ahrs -f dockerFile .
    ```
2. Run Container
    ```shell=
    docker run -it --rm \
      --privileged \
      --net=host \
      --ipc=host \
      -e DISPLAY=$DISPLAY \
      -e XAUTHORITY=$XAUTHORITY \
      -v $XAUTHORITY:$XAUTHORITY \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v $(pwd):/workspace \
      -v /dev:/dev \
      ahrs
    ```
3. Compile
    ```shell=
    ./build.sh
    ```
    - Online streaming mode

        modify ```build.sh``` with ```cmake .. -DBUILD_DEPTHAI_APP=ON```.
    -  Offline replay mode
        
        modify ```build.sh``` with ```cmake ..```.
4. Run
    - Online depthai streaming mode
        
        ```./build/depthai_6dof```
    - Offline replay mode
        
        ```./build/csv_9dof```