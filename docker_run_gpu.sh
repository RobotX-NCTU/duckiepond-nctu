docker run --name duckiepond --rm -it --net=host --privileged -v /dev:/dev \
			-v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock \
			-v /home/$USER:/root -v ~/.bashrc:/root/.bashrc \
			--runtime=nvidia \
			tonycar12002/duckiepond:laptop_gpu
