# RobotX-NCTU-MOOS
This is the instrcution for how to install MOOS

# Installation
1. Install Packages
```
  $ sudo apt-get install  g++  cmake  xterm 
  $ sudo apt-get install  libfltk1.3-dev  freeglut3-dev  libpng12-dev  libjpeg-dev 
  $ sudo apt-get install  libxft-dev  libxinerama-dev   libtiff5-dev
```

2. Install MOOS-IvP
```
  $ cd
  $ git clone https://github.com/ARG-NCTU/moos-ivp-nctu.git moos-ivp
  $ cd moos-ivp
  $ ./build-moos.sh
  (let it build)
  $ ./build-ivp.sh
  (let it build)
```
3. Export the MOOS-IvP path
```
  $ sudo vim ~/.bashrc
  (add this line in the bottom, "export PATH=$HOME/moos-ivp/bin:$PATH" )
  $ source ~/.bashrc
  $ which MOOSDB
  /Users/you/moos-ivp/bin/MOOSDB
  $ which pHelmIvP 
  /Users/you/moos-ivp/bin/pHelmIvP
```

4. Build our moos
```
  $ cd 
  $ cd robotx_nctu/moos
  $ ./build.sh
  $ sudo vim ~/.bashrc
  (add this line in the bottom, "export PATH=$HOME/robotx_nctu/moos/bin:$PATH" )
  $ which pXRelayTest
  /home/you/robotx_nctu/moos/bin/pXRelayTest
```





