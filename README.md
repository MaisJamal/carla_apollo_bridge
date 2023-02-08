# Carla Apollo Bridge

This python package provides a bridge for communicating between Carla's Python API and Apollo.  Besides the source code, a Dockerfile and scripts are provided for getting setup quickly and easily.  This package was tested with Carla version 0.9.13, and Apollo v7.0.0.


## Installation

### Pre-requisites

For the simplest setup, we will run Carla in Docker. 

#### docker

[https://docs.docker.com/install/linux/docker-ce/ubuntu/](https://docs.docker.com/install/linux/docker-ce/ubuntu/)

#### nvidia-docker-2

[https://github.com/nvidia/nvidia-docker-2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

## Setup / Getting Started

The following commands will be run with 3 containers:

- carla-server: this container will run the Carla simulator
- carla-apollo-13: bridge between apollo and carla-server containers, has cyber_py and carla python packages installed and unlike apollo container, can easily display gui applications on local machine
- apollo_dev_user: runs the apollo stack


### Clone and build Apollo

Clone Apollo : 

```
git clone https://github.com/ApolloAuto/apollo

```

Out of container open /apollo/cyber/setup.bash and change Cyber_ip to 172.17.0.1

```
export CYBER_IP=172.17.0.1

```

Add the following code lines to the file **apollo/modules/planning/tasks/deciders/speed\_bounds\_decider/speed\_limit\_decider.cc** after defining speed\_limit\_from\_reference\_line :

```

// (1) speed limit from map

double speed_limit_from_reference_line = reference_line_.GetSpeedLimitFromS(reference_line_s);

/************** added as a temproraly solution *********************/
/****** for speed_limit_from_reference_line = 0 in Carla Towns *****/

if (speed_limit_from_reference_line == 0) {

	speed_limit_from_reference_line = 11.176 ;
}

/*******************************************************************/
     
```

**Important:** Update controlling parameters for Carla Lincoln vehicle in Apollo:

- Replace the file **control\_conf.pb.txt** in **/apollo/modules/calibration/data/Lincoln2017MKZ\_LGSVL** by the file in **carla\_apollo\_bridge/apollo\_control**



Now in the apollo container, build apollo...
```
# run in apollo_dev_user container:

./apollo.sh build_gpu
```

Add [Carla Maps](https://github.com/MaisJamal/Carla_Towns-Apollo_maps) to Apollo.



## Usage

#### Run Apollo container and enter it

```
cd apollo
./docker/scripts/dev_start.sh
./docker/scripts/dev_into.sh

```

Start Apollo Dreamview inside the container. 

```
bash scripts/bootstrap.sh

```

In Dreamview setup the mode to **Mkz Lgsvl** , the vehicle to **Lincoln2017MKZ LGSVL** and the map to the needed Carla Town, then run the following modules: **Routing, Planning, Third party perception, Prediction**.

To monitor the planning and control process turn **PNC Monitor** on.

#### Run Carla docker container 

```
# run on local machine:

docker run -it --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /usr/lib/nvidia:/usr/lib/nvidia --device /dev/dri --rm -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia -e DISPLAY=$DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all --gpus=all --name=carla-server --net=host -d carlasim/carla:0.9.13
```


### Build docker image / run container for Carla-Apollo-13 bridge

if this is not first run, remove the old container:
```

docker rm carla-apollo-13

```

```
# run on local machine, starting from the root of this repo:

cd docker
./build_docker.sh
./run_docker.sh


# enter Carla-apollo-13 container

docker exec -ti carla-apollo-13 bash


```

Change CYBER_IP in /apollo/cyber/setup.bash to the carla-apollo-13 container IP address

To find out the ip address to use, run this command outside of the container:

```

# get ip address of carla-apollo-13 container (default: 172.17.0.2):

docker inspect carla-apollo-13 | grep IPAddress


# in carla-apollo-13 container change the Cyber_ip:

gedit /apollo/cyber/setup.bash


# then source your ~/.bashrc file to apply the changes:
source ~/.bashrc


```


#### Create an ego vehicle and client

Choose one of Carla maps in Apollo dreamview , then change the map in UnrealEngine to it by:

```
python carla-python-0.9.13/util/config.py -m Town03 --host 172.17.0.1


```

Run these commands inside the carla-apollo-13 container

```
# run in carla-apollo-13 container, start Carla example scenario:

cd ~/carla_apollo_bridge_13

python examples/manual_control_13.py 

```

Before starting the bridge set the configurations in config/bridge\_settings.yaml. To enable the control from apollo, turn on the **control module** in Apollo and in the settings file set apply\_control to true , otherwise the ego vehicle in Carla will follow the planned trajectory without applying control.

```
# in config/bridge_settings.yaml: 

apply_control : true

```

To publish GroundTruth of obstacles:

```
# in config/bridge_settings.yaml: 

publish_obstacles_ground_truth: true

```

Run the bridge:

```
# run in carla-apollo-13 container, start carla-apollo bridge:

cd ~/carla_apollo_bridge_13

python carla_cyber_bridge/run_bridge.py

```

<center><img src="gifs/GT_obstacles.gif" width = "625"></center>


#### Interfacing with the simulation

For interfacing with the simulator, a copy of the Carla PythonAPI is included in the carla-apollo-13 container.  Some uses:

```
# run in another carla-apollo-13 container terminal:
cd ~/carla_apollo_bridge_13/

# change the map
python carla-python-0.9.13/util/config.py -m Town04 --host 172.17.0.1

# spawn traffic
python carla-python-0.9.13/examples/generate_traffic.py 

```

