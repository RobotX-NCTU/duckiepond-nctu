FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO melodic

###################################### user #####################################

ENV SHELL=/bin/bash \
    USER=teamtaiwan \
    UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER} 

RUN echo "root:root" | chpasswd
RUN echo "${USER}:teamtaiwan" | chpasswd

###################################### basic tools #####################################

RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -yq dist-upgrade \
    && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    locales \
    cmake \
    make \
    git \
    vim \
    gedit \
    wget \
    sudo \
    lsb-release \
    build-essential \
    net-tools \
    dirmngr \
    gnupg2 \
    mercurial \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libpcap-dev \
    libmetis-dev \
    python-gtk2 \
    python-gobject \
    python-tk \
    python-pip \
    python3-pip \
    python3-setuptools \
    python3-empy  \
    python3-scipy \
    python-vcstools \
    fluid \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

###################################### ROS #####################################

# setup sources.list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install bootstrap tools
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install --no-install-recommends -y \
    ros-$ROS_DISTRO-desktop-full \
    ros-$ROS_DISTRO-serial \
    ros-$ROS_DISTRO-soem \
    ros-$ROS_DISTRO-openslam-gmapping \
    ros-$ROS_DISTRO-geodesy \
    ros-$ROS_DISTRO-cartographer-* \
    ros-$ROS_DISTRO-cartographer-ros \
    ros-$ROS_DISTRO-cartographer-rviz \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    ros-$ROS_DISTRO-perception \
    ros-$ROS_DISTRO-navigation \
    ros-$ROS_DISTRO-rgbd-launch \
    ros-$ROS_DISTRO-joystick-drivers \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-spacenav-node \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-velodyne-simulator \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-hector-gazebo \
    ros-$ROS_DISTRO-hector-trajectory-server \
    ros-$ROS_DISTRO-lms1xx \
    ros-$ROS_DISTRO-rviz-imu-plugin \
    ros-$ROS_DISTRO-interactive-marker-twist-server \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-rosserial-arduino \
    ros-$ROS_DISTRO-rosserial \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-robot \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-geographic-msgs \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-web-video-server \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-gps-common \
    ros-$ROS_DISTRO-ecl-threads \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init \
    && rosdep update


##################################### PIP ######################################
RUN pip2 install --upgrade pip setuptools

RUN pip2 install \
    scipy


RUN pip3 install --upgrade pip setuptools

RUN pip3 install  \
    tensorflow \
    tables==3.5.1 \
    matplotlib \
    pandas \
    requests \
    scikit-image \
    scikit-learn \
    h5py \
    testresources \
    pyyaml \
    rospkg \
    cython \
    catkin-tools \
    gdown \
    seaborn

RUN pip3 install \
    torch==1.5.1 \
    torchvision==0.6.1

####################################### procman ###########################################

RUN cd ${HOME} && git clone https://github.com/lcm-proj/lcm \
    && cd lcm \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

RUN cd ${HOME} && git clone http://github.com/ARG-NCTU/procman \
    && cd procman \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

################################### MOOS-Ivp ###################################

RUN cd ${HOME}/ \
    && git clone https://github.com/ARG-NCTU/moos-ivp-nctu.git moos-ivp \
    && cd moos-ivp \
    && ./build-moos.sh \
    && ./build-ivp.sh

# Add MOOS-IvP to the PATH
ENV PATH "$PATH:/${HOME}/moos-ivp/bin"

# Add MOOS-IvP to the PATH
ENV PATH "$PATH:/root/moos-ivp/bin" 


################################### ZED-3.2-cuda10.2 ###################################

WORKDIR ${HOME}

RUN apt-get update -y && apt-get install --no-install-recommends less udev -y && \
    wget -O ZED_SDK_Linux_Ubuntu18.run https://download.stereolabs.com/zedsdk/3.2/cu102/ubuntu18 && \
    chmod +x ZED_SDK_Linux_Ubuntu18.run ; ./ZED_SDK_Linux_Ubuntu18.run -- silent skip_tools && \
    rm ZED_SDK_Linux_Ubuntu18.run && \
    chown -R ${USER}:${USER} /usr/local/zed && \
    rm -rf /var/lib/apt/lists/*

#################################### BASH ####################################

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc \
    && echo "cd ~/robotx_nctu" >> ${HOME}/.bashrc \
    && echo "source ~/robotx_nctu/catkin_ws/devel/setup.bash" >> ${HOME}/.bashrc

#############################################################################

RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL)  ALL" > /etc/sudoers

# Support of nvidia-docker 2.0
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

USER ${USER}

WORKDIR ${HOME}
