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
  
    docker run --rm -it --net=host --privileged -v /dev:/dev -v /etc/localtime:/etc/localtime:ro -v /home/{user_name}/duckiepond-nctu:/hostname  duckiepond:laptop 
    
  open a browser enter:
  
    localhost:80
    
  or vnc viewer:
  
    localhost:5900
    
  you should see the desktop of vnc
    
---------------------------------------------------------------------------------------------------------
 
on pi3:
---

  1.build yourself:
  
    cd duckiepond-nctu/docker/pi3
    docker build --rm -t duckiepond:pi3 . --no-cache
    
  2.pull from dockerhub:
  
    docker pull juite/duckiepond:rpi
    docker tag juite/duckiepond:laptop duckiepond:pi3
    
  run on pi3:
  
    docker run --rm -it --net=host --privileged -v /dev:/dev duckiepond:pi3
