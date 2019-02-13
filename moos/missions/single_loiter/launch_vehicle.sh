#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
JUST_BUILD="no"
SHORE_PORT="9000"
VEH_PORT="9100"

VNAME="BRIAN"
SHORE_IP="192.168.2.105"
BRIAN_IP="192.168.2.101"
MONICA_IP="192.168.2.111"
VEH_IP=$BRIAN_IP

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        printf "%s [SWITCHES] [time_warp]   \n" $0
        printf "  --shore=IP address of shoreside       \n"  
        printf "  --just_make, -j    \n" 
        printf "  --vname=VNAME      \n" 
        printf "  --brian, -b         \n" 
        printf "  --monica, -m         \n" 
        printf "  --help, -h         \n" 
        exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
        echo $TIME_WARP
    elif [ "${ARGI:0:11}" = "--shore_ip=" ] ; then
        SHORE_IP="${ARGI#--shore_ip=*}"
    elif [ "${ARGI:0:9}" = "--veh_ip=" ] ; then
        VEH_IP="${ARGI#--veh_ip=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ] ; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI}" = "--brian" -o "${ARGI}" = "-b" ] ; then
        VNAME="BRIAN"
        VEH_IP=$BRIAN_IP
    elif [ "${ARGI}" = "--monica" -o "${ARGI}" = "-m" ] ; then 
        VNAME="MONICA"
        VEH_IP=$MONICA_IP
    elif [ "${ARGI}" = "--just_build" -o "${ARGI}" = "-j" ] ; then
        JUST_BUILD="yes"
    else 
        printf "Bad Argument: %s \n" $ARGI
        exit 0
    fi
done


#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
LOITER_POS="0,0"

nsplug vehicle.moos duckiepond_$VNAME.moos -f \
    VNAME=$VNAME        VEH_PORT=$VEH_PORT    \
    VEH_IP=$VEH_IP      SHORE_IP=$SHORE_IP \
    SHORE_PORT=$SHORE_PORT START_POS=$START_POS

nsplug vehicle.bhv duckiepond_$VNAME.bhv -f VNAME=$VNAME     \
START_POS=$START_POS


if [ ${JUST_BUILD} = "yes" ] ; then
    exit 0
fi


#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------

printf "Launching the %s MOOS Community (WARP=%s) \n"  duckiepond_$VNAME $TIME_WARP
pAntler duckiepond_$VNAME.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &

uMAC duckiepond_$VNAME.moos


printf "Killing all processes ... \n"
kill %1 
mykill
printf "Done killing processes.   \n"
