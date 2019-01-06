#!/bin/bash -e
SHORE_IP="localhost"
SHORE_LISTEN="9000"
VEH1_IP="localhost"
VEH1_LISTEN="9100"
VEH2_IP="localhost"
VEH2_LISTEN="9200"

COMMUNITY="shoreside"

#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
SHORE_IP=$1
echo "SHORE_IP = " $1

VEH1_IP=$1
echo "VEH1_IP = " $2

VEH2_IP=$1
echo "VEH2_IP = " $3

#-------------------------------------------------------
#  Part 2: Launch the processes
#-------------------------------------------------------
nsplug ${COMMUNITY}.moos duckiepond_${COMMUNITY}.moos -f  \
    SHORE_IP=$SHORE_IP \
    SHORE_LISTEN=$SHORE_LISTEN \
    VEH1_IP=$VEH1_IP \
    VEH1_LISTEN=$VEH1_LISTEN \
    VEH2_IP=$VEH2_IP \
    VEH2_LISTEN=$VEH2_LISTEN 


printf "Launching the %s MOOS Community (WARP=%s) \n"  duckiepond_${COMMUNITY} $TIME_WARP
pAntler duckiepond_${COMMUNITY}.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &
uMAC -t duckiepond_${COMMUNITY}.moos

printf "Killing all processes ... \n"
kill %1 
mykill
printf "Done killing processes.   \n"
