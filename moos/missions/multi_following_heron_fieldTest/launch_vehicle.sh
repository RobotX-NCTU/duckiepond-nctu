#!/bin/bash -e
SHORE_IP="192.168.2.105"
SHORE_PORT="9000"
SHORE_LISTEN="9200"

VEH_NAME1="BRIAN"
VEH_IP1="192.168.2.101"
VEH_PORT1="9001"
VEH_LISTEN1="9201"

VEH_NAME2="MONICA"
VEH_IP2="192.168.2.111"
VEH_PORT2="9002"
VEH_LISTEN2="9202"

VEH_NAME3="ALENANDER"
VEH_IP3="192.168.2.113"
VEH_PORT3="9003"
VEH_LISTEN3="9203"
M200_IP="192.168.2.1"

COMMUNITY="shoreside"
SIM="true"

START_POS1="-20,0"         
START_POS2="0,0"
START_POS3="20,0"
#SIM="false"

#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
echo "SHORE_IP = " $SHORE_IP ", VEH_IP1 = " $VEH_IP1 ", VEH_IP2 = " $VEH_IP2 ", VEH_IP3 = " $VEH_IP3
echo "SHORE_PORT = " $SHORE_PORT ", VEH_PORT1 = " $VEH_PORT1 ", VEH_PORT2 = " $VEH_PORT2", VEH_PORT3 = " $VEH_PORT3

#-------------------------------------------------------
#  Part 2: Launch the processes
#-------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        HELP="yes"
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI

    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ] ; then
        SIM="true"
        echo "Simulation mode ON"   

    elif [ "${ARGI}" = "--brian" -o "${ARGI}" = "-b" ] ; then 
        nsplug vehicle.moos duckiepond_$VEH_NAME1.moos -f \
            VNAME=$VEH_NAME1  VEH_PORT=$VEH_PORT1    \
            VEH_IP=$VEH_IP1      SHORE_IP=$SHORE_IP \
            SHORE_PORT=$SHORE_PORT START_POS=$START_POS1 \
            VEH_LISTEN=$VEH_LISTEN1 \
            SIM=$SIM \
            SHORE_LISTEN=$SHORE_LISTEN

        nsplug vehicle.bhv duckiepond_$VEH_NAME1.bhv -f VNAME=$VEH_NAME1     \
        START_POS=$START_POS1  SHADOW="true" CONTACT_VEH="ALENANDER"

        printf "Launching the %s MOOS Community (WARP=%s) \n"  duckiepond_$VEH_NAME1 $TIME_WARP
        pAntler duckiepond_$VEH_NAME1.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &

        uMAC duckiepond_$VEH_NAME1.moos
    
    elif [ "${ARGI}" = "--monica" -o "${ARGI}" = "-m" ] ; then 

        nsplug vehicle.moos duckiepond_$VEH_NAME2.moos -f \
            VNAME=$VEH_NAME2    VEH_PORT=$VEH_PORT2 \
            VEH_IP=$VEH_IP2      SHORE_IP=$SHORE_IP \
            SHORE_PORT=$SHORE_PORT START_POS=$START_POS2 \
            VEH_LISTEN=$VEH_LISTEN2\
            SIM=$SIM \
            SHORE_LISTEN=$SHORE_LISTEN

        nsplug vehicle.bhv duckiepond_$VEH_NAME2.bhv -f VNAME=$VEH_NAME2     \
            START_POS=$START_POS2   SHADOW="true" CONTACT_VEH="BRIAN"

        printf "Launching the %s MOOS Community (WARP=%s) \n"  duckiepond_$VEH_NAME2 $TIME_WARP
        pAntler duckiepond_$VEH_NAME2.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &

        uMAC duckiepond_$VEH_NAME2.moos
    
    elif [ "${ARGI}" = "--alexander" -o "${ARGI}" = "-a" ] ; then 

        nsplug vehicle.moos duckiepond_$VEH_NAME3.moos -f \
            VNAME=$VEH_NAME3    VEH_PORT=$VEH_PORT3 \
            VEH_IP=$VEH_IP3      SHORE_IP=$SHORE_IP \
            SHORE_PORT=$SHORE_PORT START_POS=$START_POS3 \
            VEH_LISTEN=$VEH_LISTEN3\
            SIM=$SIM \
            SHORE_LISTEN=$SHORE_LISTEN M200_IP=$M200_IP

        nsplug vehicle.bhv duckiepond_$VEH_NAME3.bhv -f VNAME=$VEH_NAME3     \
            START_POS=$START_POS3  SHADOW="false" CONTACT_VEH="ALENANDER"

        printf "Launching the %s MOOS Community (WARP=%s) \n"  duckiepond_$VEH_NAME3 $TIME_WARP
        pAntler duckiepond_$VEH_NAME3.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &

        uMAC duckiepond_$VEH_NAME3.moos
    fi
done

printf "Killing all processes ... \n"
kill %1 
mykill
printf "Done killing processes.   \n"
