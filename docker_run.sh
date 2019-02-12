#!/usr/bin/env bash
if [ $# -gt 0 ]; then
	if [ "$1" == "same" ]; then
		docker exec -it duckiepond bash
	else
		docker run --name duckiepond --rm -it --net=host --privileged -v /dev:/dev \
			-v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock \
			-v /home/$USER/duckiepond-nctu:/root/duckiepond-nctu -v ~/.bashrc:/root/.bashrc \
			 juite/duckiepond:$1      
	fi
else
	echo "please provide docker tag name."
fi
