FROM nvidia/cudagl:9.0-devel-ubuntu16.04

# ros
# install bootstrap tools
RUN apt-get update -y
RUN apt-get install -y bash git make wget sudo build-essential \
    cmake net-tools iputils-ping network-manager \
    vim python-pip python-dev \
    libjpeg-dev libtiff-dev libpng-dev libfltk1.3-dev fluid \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*


# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

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
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-gazebo-ros \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-xacro \
    ros-kinetic-gps-common \
    ros-kinetic-hector-gazebo \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get install -y wget python-pip python-dev libgtk2.0-0 unzip libblas-dev liblapack-dev libhdf5-dev && \
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python get-pip.py

RUN  pip install scikit-learn 
RUN  pip install setuptools==40.6.3
RUN  pip install --ignore-installed tensorflow-gpu==1.12.0rc1 matplotlib pandas scipy h5py testresources 

RUN wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1WPrDS29I9l7r6lAdW-cpH-tDc0CW1tgc' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1WPrDS29I9l7r6lAdW-cpH-tDc0CW1tgc" -O cudnn-9.0-linux-x64-v7.4.2.24.tar.gz && rm -rf /tmp/cookies.txt
RUN tar zxvf cudnn-9.0-linux-x64-v7.4.2.24.tar.gz 
RUN cp -P cuda/include/cudnn.h /usr/local/cuda-9.0/include
RUN cp -P cuda/lib64/libcudnn* /usr/local/cuda-9.0/lib64/
RUN chmod a+r /usr/local/cuda-9.0/lib64/libcudnn*

RUN rm -rf cudnn-9.0-linux-x64-v7.4.2.24.tar.gz
WORKDIR /root

RUN pip install tables==3.5.1 

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
WORKDIR /root

RUN apt-get install -y byobu
RUN wget -r -R "index\.html*" http://models.gazebosim.org/
RUN mkdir .gazebo
RUN mv models.gazebosim.org/ .gazebo/models


# Install Caffe
WORKDIR /root
RUN git clone https://github.com/tonycar12002/caffe.git && \
    cd caffe && git checkout ssd

RUN apt-get install -y libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
RUN apt-get install -y --no-install-recommends libboost-all-dev
RUN apt-get install -y libopenblas-dev liblapack-dev libatlas-base-dev
RUN apt-get install -y libgflags-dev libgoogle-glog-dev liblmdb-dev
RUN apt-get install -y git cmake build-essential

WORKDIR /root
RUN cd caffe && git pull && cp Makefile.config.example Makefile.config && make -j8
RUN cd caffe && make py
RUN cd caffe && make test -j8
RUN touch /root/.bashrc && echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc
RUN touch /root/.bashrc && echo export PYTHONPATH="/root/caffe/python" >> /root/.bashrc
RUN pip install scikit-image
RUN apt-get install -y protobuf-compiler python-pil python-lxml python-tk
RUN apt-get install -y pyqt4-dev-tools pyqt5-dev-tools && pip install lxml
