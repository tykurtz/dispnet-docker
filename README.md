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
