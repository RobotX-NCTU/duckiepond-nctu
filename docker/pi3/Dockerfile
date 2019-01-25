FROM argnctu/ncs_ver1_duckietown_based

#install essential package
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update -y && apt-get install -y \
    bash git cmake make wget sudo \
    python-pip python-dev build-essential \
    cmake net-tools iputils-ping network-manager vim \
    libjpeg-dev libtiff-dev libpng-dev libfltk1.3-dev fluid \
    ros-kinetic-rosserial-arduino \
    ros-kinetic-rosserial \
    ros-kinetic-geodesy \
    && rm -rf /var/lib/apt/lists/*

RUN sudo pip install utm

#install moos-ivp
WORKDIR /root
RUN git clone https://github.com/ARG-NCTU/moos-ivp-nctu.git moos-ivp 
WORKDIR /root/moos-ivp
RUN ./build-moos.sh
RUN ./build-ivp.sh

# Add MOOS-IvP to the PATH
ENV PATH "$PATH:/root/moos-ivp/bin" 

WORKDIR /root
ENV USER root
#install rpi gui
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    xserver-xorg xinit

RUN apt-get install -y\
    mate-desktop-environment-core\
    lightdm tightvncserver\
    && rm -rf /var/lib/apt/lists/*


