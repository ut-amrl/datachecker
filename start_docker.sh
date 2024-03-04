#!/bin/bash
docker run --net=host -it \
    -v `pwd`:/home/datachecker \
    --shm-size 16g datachecker\
