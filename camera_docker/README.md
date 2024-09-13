# ZED ROS Noetic Docker Container

Here are the necessary files in order to run a Docker container specific for ZED camera with ROS Noetic.
<br>

## Prerequisites

1. Install Docker
    ```bash
      curl -sSL https://get.docker.com/ | sh
    ```

2. Make sure docker is installed correctly
    ```bash
      sudo docker run hello-world
    ```
   The output should be:
    ```bash
      Hello from Docker!
      This message shows that your installation appears to be working correctly.
      
      To generate this message, Docker took the following steps:
       1. The Docker client contacted the Docker daemon.
       2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
          (amd64)
       3. The Docker daemon created a new container from that image which runs the
          executable that produces the output you are currently reading.
       4. The Docker daemon streamed that output to the Docker client, which sent it
          to your terminal.
      
      To try something more ambitious, you can run an Ubuntu container with:
       $ docker run -it ubuntu bash
      
      Share images, automate workflows, and more with a free Docker ID:
       https://hub.docker.com/
      
      For more examples and ideas, visit:
       https://docs.docker.com/get-started/
    ```
<br>

## Install ZED Docker Container

1. Open a new terminal and navigate to the [ZED_Docker](/ZED_Docker) directory

2. Build the provided [Dockerfile](/ZED_Docker/Dockerfile)
    ```bash
      docker build -t camera:v1.0 .
    ```

3. Create and start the ZED Docker Container
    ```bash
       ./create_camera_container.sh
    ```
Note: To reopen the container later, use:
    ```bash
       ./start_camera_container.sh
    ```

<br>

## Add ZED Docker Container icon to Shadow software

 - Copy the items into [add2shadow](/camera_docker/add2shadow) folder to the respective Shadow software folders.

 - In the [CAMERA Container.desktop](/camera_docker/add2shadow/Shadow%20Icons/CAMERA%20Container.desktop) file change the "user" to your pc username so that the icon appears correctly.
<br>

## Push to Docker

If you want to save your changed Docker Container into your Docker repository, follow the next steps:

1. Login into your Docker account
    ```bash
        docker login
    ```
    
2. Commit changes
    ```bash
       docker commit <container_id> <image_name>:<image_tag>
    ```

3. Tag the image
    ```bash
       docker tag <image_name>:<image_tag> <docker_user>/<image_name>:<image_tag>
    ```

4. Push to Docker
    ```bash
       docker push <docker_user>/<image_name>:<image_tag>
    ```
<br>

## Useful Docker commands

 - List all Docker Images
    ```bash
       docker image ls -a
    ```
    
 - Remove a Docker Image
    ```bash
       docker rmi <docker_image_name>
    ```
   
 - List all Docker Containers
    ```bash
       docker container ls -a
    ```
    
 - Remove a Docker Container
    ```bash
       docker rm <docker_container_name>
    ```
