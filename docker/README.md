Docker develop environment
---

on laptop:
--

  1.build yourself:
  
    cd duckiepond-nctu/docker/laptop
    docker build --rm -t duckiepond:laptop . --no-cache
    
  2.pull from dockerhub:
  
    docker pull juite/duckiepond:laptop
    docker tag juite/duckiepond:laptop duckiepond:laptop
    
  run on laptop:
  
    docker run --rm -it --net=host --privileged -v /dev:/dev -v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock -v /home/{user_name}/duckiepond-nctu:/hostname  duckiepond:laptop 
    
  open a browser enter:
  
    localhost:80
    
  or vnc viewer:
  
    localhost:5900
    
  you should see the desktop of vnc
  
---------------------------------------------------------------------------------------------------------
 
on tx2:
---

  1.build yourself:
  
    cd duckiepond-nctu/docker/tx2
    docker build --rm -t duckiepond:tx2 . --no-cache
    
  2.pull from dockerhub:
  
    docker pull juite/duckiepond:tx2
    docker tag juite/duckiepond:tx2  duckiepond:tx2
    
  run on tx2:
  
    docker run --rm -it --net=host --privileged -v /dev:/dev -v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock -v /home/{user_name}/duckiepond-nctu:/hostname duckiepond:tx2
    
---------------------------------------------------------------------------------------------------------
 
on pi3:
---

  1.build yourself:
  
    cd duckiepond-nctu/docker/pi3
    docker build --rm -t duckiepond:pi3 . --no-cache
    
  2.pull from dockerhub:
  
    docker pull juite/duckiepond:rpi
    docker tag juite/duckiepond:rpi  duckiepond:pi3
    
  run on pi3:
  
    docker run --rm -it --net=host --privileged -v /dev:/dev -v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock -v /home/{user_name}/duckiepond-nctu:/hostname duckiepond:pi3
