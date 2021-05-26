# Building ros docker image
*You should run this code on a PC with Graphics Processing Unit(GPU)*

## Environment
- Ubuntu 18.04
- ROS melodic 
- CUDA 10.1
- python3
- PCL (Point Cloud Library)
- GTSAM
- ISAM
- pytorch 1.5.0
- OpenCV 3.3.1
- realsense library
- MOOS-IvP

**building docker image**
```
source build.sh
```

# Run docker

**enter docker container**
```
$ source /YOUR_WORKING_DIR/robotx_nctu/docker_run.sh
```

**join docker container from another terminal**
```
$ source /YOUR_WORKING_DIR/robotx_nctu/docker_join.sh
```