if [ $# -gt 0 ]; then
	if [ "$1" == "same" ]; then
		docker exec -it duckiepond bash
	else
		docker run --name duckiepond --rm -it --net=host --privileged -v /dev:/dev \
			-v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock \
			-v /home/$USER/duckiepond-nctu:/root/duckiepond-nctu \
			-v /home/$USER/duckiepond_gazebo:/root/duckiepond_gazebo \
			-v /home/$USER/duckiepond-imitaion-learning:/root/duckiepond-imitaion-learning \
			--runtime=nvidia \
			--env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
			--env QT_X11_NO_MITSHM=1 \
			tonycar12002/duckiepond:laptop_gpu     
	fi
else
	echo "please provide docker tag name."
fi