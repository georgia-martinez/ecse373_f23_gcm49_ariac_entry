# ECSE 373 Laboratory #5

## Set up

This package relies on the ARIAC 2019 environment which you can read more about [here] (https://bitbucket.org/osrf/ariac/wiki/2019/Home). You should be using ROS noetic. 

Follow these instructions to create two separate workspaces:

```
# Create a catkin workspace for the simulation environment
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws/src

# Clone the repository
git clone https://github.com/cwru-eecs-373/cwru_ariac_2019.git

# Install any missing dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build the simulator environment
cd ../

# Install the simulator environment
sudo -- /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install"

# Make a workspace for the ARIAC node.
mkdir -p ~/ecse_373_ariac_ws/src
cd ~/ecse_373_ariac_ws/src

# Clone the GIT repository for this laboratory
git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git

# Install any missing dependencies
rosdep install --from-paths ecse_373_ariac --ignore-src -r -y

# Add it to your ROS environment
cd ../
catkin_make
source devel/setup.bash
```

## Launching

`roslaunch ariac_entry entry.launch`

When the gazebo window opens, hit the play button at the bottom of the screen to start the simulation.

## About the package

The purpose of this package is to process orders in the ARIAC simulation by:

- Subscribing to the `/ariac/orders` topic to receive new orders which are pushed to a queue
- Using the `material_location` service to find the bin(s) that has a part of the type
required by the first product in the first shipment of the first order.
- Subscribing to all logical_cameras and storing the information
- Logging a message with the bin number and (x, y, Z) position of the part
