## docker setup
1. Building image
    ```shell=
    sudo docker build --network=host -t oak -f dockerFile .
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
      oak
    ```
3. Compile
    ```shell=
    ./build.sh
    ```