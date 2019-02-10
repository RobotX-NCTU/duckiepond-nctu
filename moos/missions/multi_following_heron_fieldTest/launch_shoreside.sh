#!/bin/bash -e
SHORE_IP="localhost"
SHORE_PORT="9000"

VEH_NAME1="BRIAN"
VEH_IP1="localhost"
VEH_PORT1="9100"

VEH_NAME2="MONICA"
VEH_IP2="localhost"
VEH_PORT2="9200"

VEH_NAME3="ALENANDER"
VEH_IP3="localhost"
VEH_PORT3="9300"

COMMUNITY="shoreside"

START_POS1="-20,0"         
START_POS2="0,0"
START_POS3="20,0"

#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
echo "SHORE_IP = " $SHORE_IP ", VEH_IP1 = " $VEH_IP1 ", VEH_IP2 = " $VEH_IP2 ", VEH_IP3 = " $VEH_IP3
echo "SHORE_PORT = " $SHORE_PORT ", VEH_PORT1 = " $VEH_PORT1 ", VEH_PORT2 = " $VEH_PORT2", VEH_PORT3 = " $VEH_PORT3

#-------------------------------------------------------
#  Part 2: Launch the processes
#-------------------------------------------------------
nsplug shoreside.moos duckiepond_${COMMUNITY}.moos -f  \
    SHORE_PORT=$SHORE_PORT  SHORE_IP=$SHORE_IP \
    VEH_IP1=$VEH_IP1 VEH_IP2=$VEH_IP2 VEH_IP3=$VEH_IP3\
    VEH_PORT1=$VEH_PORT1 VEH_PORT2=$VEH_PORT2 VEH_PORT3=$VEH_PORT3\
    VEH_NAME1=$VEH_NAME1 VEH_NAME2=$VEH_NAME2 VEH_NAME3=$VEH_NAME3\
    MONICA_FOLLOW_VEH="BRIAN" BRIAN_FOLLOW_VEH="ALENANDER" ALENANDER_FOLLOW_VEH="No"


printf "Launching the %s MOOS Community (WARP=%s) \n"  duckiepond_${COMMUNITY} $TIME_WARP
pAntler duckiepond_${COMMUNITY}.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &
uMAC -t duckiepond_${COMMUNITY}.moos

printf "Killing all processes ... \n"
kill %1 
mykill
printf "Done killing processes.   \n"
