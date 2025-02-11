# easy_handeye: automated Hand-Eye Calibration for ROS1
## (noetic, UR5, realsense camera D435)

<img src="docs/img/eye_on_base_ndi_pic.png" width="345"/> <img src="docs/img/05_calibrated_rviz.png" width="475"/> 


This package provides functionality and a GUI to: 
- **sample** the robot position and tracking system output via `tf`,
- **compute** the eye-on-base or eye-in-hand calibration matrix through the OpenCV library's Tsai-Lenz algorithm 
implementation,
- **store** the result of the calibration,
- **publish** the result of the calibration procedure as a `tf` transform at each subsequent system startup,
- (optional) automatically **move** a robot around a starting pose via `MoveIt!` to acquire the samples. 


## Use Cases

If you are unfamiliar with Tsai's hand-eye calibration [1], it can be used in two ways:

- **eye-in-hand** to compute the static transform between the reference frames of
  a robot's hand effector and that of a tracking system, e.g. the optical frame
  of an RGB camera used to track AR markers. In this case, the camera is
  mounted on the end-effector, and you place the visual target so that it is
  fixed relative to the base of the robot; for example, you can place an AR marker on a table.
- **eye-on-base** to compute the static transform from a robot's base to a tracking system, e.g. the
  optical frame of a camera standing on a tripod next to the robot. In this case you can attach a marker,
  e.g. an AR marker, to the end-effector of the robot.


eye-on-base             |  eye-on-hand
:-------------------------:|:-------------------------:
![](docs/img/eye_on_base_aruco_pic.png)  |  ![](docs/img/eye_on_hand_aruco_pic.png)

## Getting started

- clone this repository into your catkin workspace:
```
cd ~/catkin_ws/src  # replace with path to your workspace
git clone https://github.com/IFL-CAMP/easy_handeye
```

- satisfy dependencies
```
cd ..  # now we are inside ~/catkin_ws
rosdep install -iyr --from-paths src
```

- build
```
catkin build
```

## Usage

### First we need to setup Moveit! to control UR robot

#### install some nessary component

```bash
sudo apt-get install ros-noetic-effort-controllers
sudo apt-get install ros-noetic-rqt-controller-manager
sudo apt-get install ros-noetic-moveit-visual-tools
sudo apt-get install ros-noetic-rviz-visual-tools
sudo apt-get install ros-noetic-trac-ik-kinematics-plugin
sudo apt-get install ros-noetic-ompl
sudo apt-get install ros-noetic-moveit-planners-ompl
```
#### get UR drivers
go to your workspace and get
```bash
cd ~/catkin_ws/src 
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
git clone -b noetic https://github.com/ros-industrial/universal_robot.git
cd ..
catkin_make
source devel/setup.bash
```

#### install Moveit!

```bash
sudo apt install ros-noetic-moveit
```

#### download external controller into UR robot
from this website (https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).
I'm using externalcontrol-1.0.3.urcap.

#### change wired network setting

I'm using Ipv4, address: 192.168.1.86 (since my UR IP address is 192.168.1.60). Netmask: 255.255.255.0 (same as UR)

#### check connection between UR robot and PC
connect wire with UR robot and PC (better turn off wifi)
```bash
ping 192.168.1.60
```

#### running UR robot with Moveit!
In three separate terminals run:
```bash
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.1.60 limited:=true
```
```bash
roslaunch ur5_moveit_config moveit_planning_execution.launch
```
```bash
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```
After opening rviz, add MotionPlanning.

#### running external control inside UR5 robot
On UR5 panel open an empty program, select external control. check your PC ip address not overlap UR5 external control host ip address. Then shut down the firewall in your PC with this command:
```bash
sudo systemctl stop firewalld
```
Finally, tap the play button (botton left of the UR panel) inside external control. 

#### Play with UR5
Now you can move your UR5. Dragging the ball (not recommended, the joints will be messed up) or pulling the joint values in the joints, click PLAN to see the expected trajectory and EXECUTE to execute the trajectory!


### eye-in-hand
Now we can start hand-eye-calibration

#### download necessary package
```bash
cd ~/catkin_ws/src
sudo apt-get install ros-noetic-visp
git clone -b noetic-devel https://github.com/pal-robotics/aruco_ros
cd ..
catkin_make
source devel/setup.bash
```
```bash
pip install transforms3d
```
don't use conda in this step, ROS1 doesn't recognise conda packages.

#### prepare aruco marker
we can generate aruco marker from here (https://chev.me/arucogen/), choose original aruco, and choose parameter (edit in my_calibration.launch). My aruco marker is ID:571, 100mm.

#### configure realsense camera
```bash
# you can replace $ROS_DISTRO by noetic
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```
and you can trying to launch realsense camera to test
```bash
roslaunch realsense2_camera rs_camera.launch
```
#### hand-eye-calibration
You can launch my calibration file directly, if you're using Ubuntu20, ROS noetic, UR5, realsense camera(D435).

```bash
roslaunch easy_handeye my_calibration.launch
```
There will be three user interfaces after launch.
1. change coordinate system to base.
2. add topic published by aruco_ros/result image.

Then, the whole calibration steps like following:
Use the panel to set the robot position, click check starting pose, if ready then open the external control in the panel to connect the robot (according to my experience, arm can be a little bit back, make aruco marker at the middle of the image, so that the success rate of the check is a little higher)

 the order is like this: next pose-plan-execute-take sample-next pose cycle, during which execute will drive the robot arm to reach the new position (if the robot arm doesn't move, that means the external control is not turned on, turn it on again and you can continue). In the middle of the bad continue next pose, a few less is not a big problem. Until the progress bar reaches 100%, and then click on the compute can see that the desired results have been calculated.

 click save, end of the story :D


<img src="docs/img/04_plan_show.png" width="495"/>

#### Tips for accuracy

The following tips are given in [1], paragraph 1.3.2.

- Maximize rotation between poses.
- Minimize the distance from the target to the camera of the tracking system.
- Minimize the translation between poses.
- Use redundant poses.
- Calibrate the camera intrinsics if necessary / applicable.
- Calibrate the robot if necessary / applicable.

### Publishing
The `publish.launch` starts a node that publishes the transformation found during calibration in `tf`.
The parameters are automatically loaded from the yaml file, according to the specified namespace.
For convenience, you can include this file within your own launch script. You can include this file multiple times to 
publish many calibrations simultaneously; the following example publishes one eye-on-base and one eye-in-hand calibration:
```xml
<launch>
  <!-- (start your robot's MoveIt! stack, e.g. include its moveit_planning_execution.launch) -->
  <!-- (start your tracking system's ROS driver) -->

  <include file="$(find easy_handeye)/launch/publish.launch">
    <arg name="namespace_prefix" value="my_eob_calib"/> <!-- use the same namespace that you used during calibration! -->
  </include>
  <include file="$(find easy_handeye)/launch/publish.launch">
    <arg name="namespace_prefix" value="my_eih_calib"/> <!-- use the same namespace that you used during calibration! -->
  </include>
</launch>
```
You can have any number of calibrations at once (provided you specify distinct namespaces). 
If you perform again any calibration, you do not need to do anything: the next time you start the system, 
the publisher will automatically fetch the latest information. You can also manually restart the publisher 
nodes (e.g. with `rqt_launch`), if you don't want to shut down the whole system.

### FAQ
#### Why is the calibration wrong?
Please check the [troubleshooting](docs/troubleshooting.md)

#### How can I ...
##### Calibrate an RGBD camera (e.g. Kinect, Xtion, ...) with a robot for automatic object collision avoidance with MoveIt! ?
This is a perfect example of an eye-on-base calibration. You can take a look at this [example launch file](docs/example_launch/ur5_kinect_calibration.launch) written for an UR5 and a Kinect via aruco_ros, or [example for LWR iiwa with Xtion/Kinect ](docs/example_launch/iiwa_kinect_xtion_calibration.launch).
##### Disable the automatic robotic movements GUI?
You can pass the argument `freehand_robot_movement:=true` to `calibrate.launch`.
##### Calibrate one robot against multiple tracking systems?
You can just override the `namespace` argument of `calibrate.launch` to be always different, such that they will never collide. Using the same `namespace` as argument to multiple inclusions of `publish.launch` will allow you to publish each calibration in `tf`.
##### Find the transformation between the bases of two robots?
You could perform the eye-on-base calibration against the same tracking system, and concatenate the results.
##### Find the transformation between two tracking systems?
You could perform the eye-on-base calibration against the same robot, and concatenate the results. This will work also if the tracking systems are completely different and do not use the same markers.

## References

[1] *Tsai, Roger Y., and Reimar K. Lenz. "A new technique for fully autonomous
and efficient 3D robotics hand/eye calibration." Robotics and Automation, IEEE
Transactions on 5.3 (1989): 345-358.*
