#!/usr/bin/env bash
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        printf "  --brian_pi3, -b_pi         \n" 
        printf "  --monica_pi3, -m_pi         \n"
		printf "  --brian_tx2, -b_tx         \n" 
		printf "  --monica_tx2, -m_tx         \n"  
		printf "  --laptop, -pc         \n"  
        printf "  --help, -h         \n" 
        exit 0;
    elif [ "${ARGI}" = "-b_pi" ] ; then
		docker run --rm -it --net=host --privileged -v /dev:/dev -v /etc/localtime:/etc/localtime:ro \
		 -v /var/run/docker.sock:/var/run/docker.sock -v /home/$USER/duckiepond-nctu:/root/duckiepond-nctu  \
		 juite/duckiepond:pi3

    elif [ "${ARGI}" = "-m_pi" ] ; then 
		docker run --rm -it --net=host --privileged -v /dev:/dev -v /etc/localtime:/etc/localtime:ro \
		 -v /var/run/docker.sock:/var/run/docker.sock -v /home/$USER/duckiepond-nctu:/root/duckiepond-nctu  \
		 juite/duckiepond:pi3

    elif [ "${ARGI}" = "-b_tx" ] ; then 
		docker run --rm -it --net=host --privileged -v /dev:/dev -v /etc/localtime:/etc/localtime:ro \
		 -v /var/run/docker.sock:/var/run/docker.sock -v /home/$USER/duckiepond-nctu:/root/duckiepond-nctu  \
		 juite/duckiepond:tx2

	elif [ "${ARGI}" = "-m_tx" ] ; then 
		docker run --rm -it --net=host --privileged -v /dev:/dev -v /etc/localtime:/etc/localtime:ro \
		 -v /var/run/docker.sock:/var/run/docker.sock -v /home/$USER/duckiepond-nctu:/root/duckiepond-nctu  \
		 juite/duckiepond:tx2

    elif [ "${ARGI}" = "-pc" ] ; then 
		docker run --rm -it --net=host --privileged -v /dev:/dev -v /etc/localtime:/etc/localtime:ro \
		 -v /var/run/docker.sock:/var/run/docker.sock -v /home/$USER/duckiepond-nctu:/root/duckiepond-nctu  \
		--add-host eastgod:192.168.2.103 juite/duckiepond:laptop

    else 
        printf "Bad Argument: %s \n" $ARGI
        exit 0
    fi
done