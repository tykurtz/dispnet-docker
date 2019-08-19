# DispNet3 -- in Docker
Fork of https://github.com/lmb-freiburg/netdef-docker with added ros integration

To build image from scratch run the following,

```sh
	sudo docker build                    \
                -f Dockerfile            \
                -t lmb-freiburg-netdef   \
                --build-arg uid=$UID    \
                --build-arg gid=$GROUPS \
                .
```

To run the container run the following from the command line
```sh
docker run --gpus all --rm -it --network host lmb-freiburg-netdef bash
```

When inside the container, run
```sh
roslaunch disp-wrapper-ros stereo_proc.launch
```

Note, this configuration is very specific to the realsense setup. Several things are hardcoded in dispnet_wrapper.py including the image topics, camera_info topics, stereo-baseline distance, and input image resolution. If something breaks, it's likely you will need to remap topics or change some hardcoded parameters (TODO Load these values from the parameter server)
