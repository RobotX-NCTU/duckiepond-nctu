#!/bin/bash -e
SHORE_IP="localhost"
SHORE_PORT="9000"

VEH_NAME1="brian"
VEH_IP1="localhost"
VEH_PORT1="9100"

VEH_NAME2="monica"
VEH_IP2="localhost"
VEH_PORT2="9200"

COMMUNITY="shoreside"

#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
echo "SHORE_IP = " $SHORE_IP ", VEH_IP1 = " $VEH_IP1 ", VEH_IP2 = " $VEH_IP2
echo "SHORE_PORT = " $SHORE_PORT ", VEH_PORT1 = " $VEH_PORT1 ", VEH_PORT2 = " $VEH_PORT2

#-------------------------------------------------------
#  Part 2: Launch the processes
#-------------------------------------------------------
nsplug shoreside.moos duckiepond_${COMMUNITY}.moos -f  \
    SHORE_PORT=$SHORE_PORT  SHORE_IP=$SHORE_IP \
    VEH_IP1=$VEH_IP1 VEH_IP2=$VEH_IP2 \
    VEH_PORT1=$VEH_PORT1 VEH_PORT2=$VEH_PORT2


printf "Launching the %s MOOS Community (WARP=%s) \n"  duckiepond_${COMMUNITY} $TIME_WARP
pAntler duckiepond_${COMMUNITY}.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &
uMAC -t duckiepond_${COMMUNITY}.moos

printf "Killing all processes ... \n"
kill %1 
mykill
printf "Done killing processes.   \n"
