# duckiepond-nctu


|Reference | Link |
|-------|--------|
|Duckiepond NCTU Github 	|[Dukciepond NCTU Repo](https://github.com/RobotX-NCTU/duckiepond-nctu.git)|
|NCTU ARG website		|[Assistive Robotics Group](https://arg-nctu.github.io)|
|MIT MOOS-IvP website|[MOOS-IvP](http://oceanai.mit.edu/moos-ivp/pmwiki/pmwiki.php?n=Main.HomePage)|
|MIT MOOS-Ivp Tutorial|[MOOS-Ivp Lab](http://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.HomePage2680)|


## Requirements Environment

- ROS kinetic
- Ubuntu 16.04
- OpenCV 3.3.1
- MOOS-Ivp 17.7.2

## Hardware

|Name | Type |
|-------		|--------					|
|Vehicle		|WAM-V						|
|Camera			|Pi Camera					|
|Depth Camera	|ZED						|

## Rule for this repo
1. Make your own branch and do your work on your branch
2. Your branch name should be: "**devel-[your name]**"
3. Please add [Monica Lin](https://github.com/nichinglin) as the reviewer when you fire a pull request
4. If you have any question about where you should put your code in, please slack [Monica Lin](https://github.com/nichinglin)

## Repo Architecture
![working on...]()

## Installation

### Docker Install

### Ubutnu 16.04 Install


```
$ sudo apt-get install ros-kinetic-desktop-full
```

## How to Build (Software)
1. For ROS part
```
$ cd 
$ cd $HOME/duckiepond-nctu/catkin_ws
$ catkin_make
```
2. For MOOS part
```
$ cd 
$ git clone https://github.com/ARG-NCTU/moos-ivp-nctu.git moos-ivp
$ cd moos-ivp
$ ./build-moos.sh
$ ./build-ivp.sh
$ source ~/.bashrc
$ cd $HOME/duckiepond-nctu/moos
$ ./build.sh
```

If you don't have ZED driver:
```
$ cd ~/duckiepond_nctu/catkin_ws
$ touch src/sensor/zed/zed-ros-wrapper/CATKIN_IGNORE
```
At this moment, you can build this repo without compile "zed-ros-wrapper"

If it still builds the pkg, try to remove /build and /devel, and then make again

Note:

Do the following everytime as you open new terminals

```
$ cd ~/duckiepond_nctu/
$ source environment.sh
```
