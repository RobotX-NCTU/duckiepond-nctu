FROM dorowu/ubuntu-desktop-lxde-vnc:xenial

#install 
RUN apt-get update -y
RUN apt-get install -y bash git make wget sudo build-essential \
    cmake net-tools iputils-ping network-manager \
    vim python-pip python-dev \
    libjpeg-dev libtiff-dev libpng-dev libfltk1.3-dev fluid \
    && rm -rf /var/lib/apt/lists/*

RUN sudo pip install utm

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y --fix-missing\
    ros-kinetic-ros-core=1.3.2-0* \
    ros-kinetic-desktop-full \
    ros-kinetic-robot \
    ros-kinetic-perception \
    ros-kinetic-navigation \
    ros-kinetic-robot-localization \
    ros-kinetic-roslint \
    ros-kinetic-hector-trajectory-server \
    ros-kinetic-joystick-drivers \
    ros-kinetic-rqt \
    ros-kinetic-rqt-common-plugins \
    ros-kinetic-web-video-server \
    ros-kinetic-rosserial-arduino \
    ros-kinetic-rosserial \
    ros-kinetic-geodesy \
    && rm -rf /var/lib/apt/lists/*


#install ncsdk2
WORKDIR /root
RUN mkdir ncs 
WORKDIR /root/ncs
RUN git clone -b ncsdk2 https://github.com/movidius/ncsdk.git
WORKDIR /root/ncs/ncsdk
RUN sudo -H make install

#install moos-ivp
WORKDIR /root
RUN git clone https://github.com/ARG-NCTU/moos-ivp-nctu.git moos-ivp 
WORKDIR /root/moos-ivp
RUN ./build-moos.sh
RUN ./build-ivp.sh

# Add MOOS-IvP to the PATH
ENV PATH "$PATH:/root/moos-ivp/bin" 

#start vnc on boot
ENTRYPOINT ["/startup.sh"]
CMD ["bash"]