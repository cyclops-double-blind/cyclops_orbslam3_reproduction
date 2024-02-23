# `cyclops_orbslam3_reproduction`
Docker application for ORBSLAM3 initialization experiment reproduction.

## Prerequisites
Install the following prerequisites.
* `docker`: follow [docker installation guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
  and [docker post-installation guide](https://docs.docker.com/engine/install/linux-postinstall/).
* Docker SDK for python: run `$ pip install docker`.

This repository runs as a normal ROS/catkin package. See the example launch file in `launch/example.launch`.
You may try the example launch file by the following commands.
```
$ cd <workspace_directory>
$ catkin_make
$ roslaunch orbslam3_docker example.launch
```
