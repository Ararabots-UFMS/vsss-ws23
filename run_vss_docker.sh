xhost +
docker run -it --rm --net=host --env="DISPLAY" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --privileged ararabots_vsss:latest
