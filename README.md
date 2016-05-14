# steering_behaviors_controller
Controller for mobile robots implementing steering behaviors

#### to view, test, work...do

##### http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

#### Install ROS
$ sudo apt-get install ros-indigo-desktop-full 

#### create a ROS workspace
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace

#### clone the repository
$ cd ~/catkin_ws/src/
$ git clone https://github.com/nahueJ/steering_behaviors_controller.git

#### compile
$ cd ~/catkin_ws/
$ catkin_make

#### run roscore
$ roscore

#### run stage simulator
$ rosrun stage_ros stageros /home/nahuel/catkin_ws/src/steering_behaviors_controller/world/willow-four-erratics.world 

#### run the controller
$ rosrun steering_behaviors_controller steering_behaviors_controller 
