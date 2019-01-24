#!/usr/bin/env bash
if [ $# -gt 0 ]; then
	# provided a hostname, use it as ROS_MASTER_URI
	docker run --rm -it --net=host --privileged -v /dev:/dev -v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock -v /home/$USER/duckiepond-nctu:/hostname  juite/duckiepond:$1 
else
	echo "please provide docker tag name."
fi

