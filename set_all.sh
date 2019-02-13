#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------

MASTER_PC_IP='192.168.2.105'
BRIAN_RPI_IP='192.168.2.102'
BRIAN_TX2_IP='192.168.2.101'
MONICA_RPI_IP='192.168.2.112'
MONICA_TX2_IP='192.168.2.111'
TONY_RPI_IP='192.168.2.122'
TONY_TX2_IP='192.168.2.121'
LAY_RPI_IP='192.168.2.132'
LAY_TX2_IP='192.168.2.131'

VNAME='BRIAN'
DEVICE='RPI'
VNAME_DEVICE_IP=$BRIAN_RPI_IP
CONTINUE="True"

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        printf "  --master_ip=MASTER_IP      \n" 
        printf "  --vname=VNAME      \n" 
        printf "  --brian, -b         \n" 
        printf "  --monica, -m         \n" 
        printf "  --help, -h         \n" 
        exit 0;
    elif [ "${ARGI:0:12}" = "--master_ip=" ] ; then
        MASTER_PC_IP="${ARGI#--master_ip=*}"
    elif [ "${ARGI}" = "-b" ] ; then
        VNAME='BRIAN'
    elif [ "${ARGI}" = "-m" ] ; then 
        VNAME='MONICA'
    elif [ "${ARGI}" = "-t" ] ; then 
        VNAME='TONY'
    elif [ "${ARGI}" = "-l" ] ; then 
        VNAME='LAY'
    elif [ "${ARGI}" = "-tx2" ] ; then 
        DEVICE='TX2'
    elif [ "${ARGI}" = "-rpi" ] ; then 
        DEVICE='RPI'
    elif [ "${ARGI}" = "-pc" ] ; then 
        VNAME='MASTER'
        DEVICE='PC'
    else 
        printf "Bad Argument: %s \n" $ARGIs
        return
    fi
done

IP=${VNAME}_${DEVICE}_IP
if [ $IP = 'MASTER_PC_IP' ] ; then
    VNAME_DEVICE_IP=$MASTER_PC_IP
elif [ $IP = 'MONICA_RPI_IP' ] ; then
    VNAME_DEVICE_IP=$MONICA_RPI_IP
elif [ $IP = 'MONICA_TX2_IP' ] ; then
    VNAME_DEVICE_IP=$MONICA_TX2_IP
elif [ $IP = 'BRIAN_RPI_IP' ] ; then
    VNAME_DEVICE_IP=$BRIAN_RPI_IP
elif [ $IP = 'BRIAN_TX2_IP' ] ; then
    VNAME_DEVICE_IP=$BRIAN_TX2_IP
elif [ $IP = 'TONY_RPI_IP' ] ; then
    VNAME_DEVICE_IP=$TONY_RPI_IP
elif [ $IP = 'TONY_TX2_IP' ] ; then
    VNAME_DEVICE_IP=$TONY_TX2_IP
elif [ $IP = 'LAY_RPI_IP' ] ; then
    VNAME_DEVICE_IP=$LAY_RPI_IP
elif [ $IP = 'LAY_TX2_IP' ] ; then
    VNAME_DEVICE_IP=$LAY_TX2_IP
else 
    printf "Bad VNAME_DEVICE_IP: %s \n" $ARGI
    return
fi

#-------------------------------------------------------
#  Part 2: Set Environments
#-------------------------------------------------------

cd ~/duckiepond-nctu
source ~/duckiepond-nctu/set_ros_master.sh $MASTER_PC_IP
source ~/duckiepond-nctu/set_ros_ip.sh $VNAME_DEVICE_IP
source ~/duckiepond-nctu/environment.sh
cd -


#-------------------------------------------------------
#  Part 3: Print useful message
#-------------------------------------------------------

echo '-------------------------------------'
printf "Source environment and set ROS_MASTER, ROS_IP\n"
printf "MASTER_IP: %s \n" $MASTER_PC_IP
printf "MACHINE: %s (%s) \n" $VNAME $DEVICE
printf "MACHINE_IP: %s \n"  $VNAME_DEVICE_IP
echo '-------------------------------------'
