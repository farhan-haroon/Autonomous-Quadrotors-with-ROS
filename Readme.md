# Autonomous Quadrotors with ROS

In this repository, I present the required methodology and code to implement a teleoperation task on an ArduPilot based Quadrotor. The Quadrotor is flying indoors localising using the **OptiTrack** motion capture system by tracking a group of infra-red reflective markers placed asymmetrically on the UAV. 

The Quadrotor is connected with the comapnion PC via the **SiK Radio 433 MHz Telemetry v2** module with it's **AirRate** setting set to the maximum value [refer to this](https://ardupilot.org/copter/docs/common-sik-telemetry-radio.html). This alows tranmitting pose data from companion PC over to the FCU with least latency. 

## Pose data transmission

The Quadrotor pose data from the OptiTrack system can be relayed to the companion PC running ROS-Noetic on Linux through an ehternet connection to the Windows PC running [Motive](https://optitrack.com/software/motive/) software. The Motive software supports various trnamission protocols like **NatNet SDK** or the [VRPN](http://wiki.ros.org/vrpn_client_ros) package. But before that, make sure that the Quadrotor is configured properly to recieve pose data from OptiTrack over **MAVLINK** by following [these](https://ardupilot.org/copter/docs/common-optitrack.html) steps.

Launch the VRPN node by the following command:

```
roslaunch vrpn_client_ros sample.launch server:=[ethernet IP of the Windows PC]
```

Clone and build this repository locally in a workspace:
```
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/ && git clone https://github.com/farhan-haroon/Autonomous-Quadrotors-with-ROS.git
cd .. && catkin_make
```

Source the wokspace in `~/.bashrc` by adding at the end: 
```
source ~/catkin_ws/devel/setup.bash
```

Once the pose data is recieved on the topic `/vrpn_client_node/<your_motive_asset_name>/pose`, we can relay this data over to the FCU via the telemetry connection by publishing the data on this topic on the topic - `/mavros/vision_pose/pose`. Relay the pose data to **MAVROS** by the following command:

```
rosrun autopilot remap.py
```

This makes the UAV localise indoors using the pose data from the OptiTrack motion capture system. Now, the UAV can be controlled by publishing setpoint velocities in the body frame of the Quadrotor on the topic - `/mavros/setpoint_raw/local`.


## Teleoperation and Circular trajectory

The teleoperation as well as the circular trajectory nodes utilize the pose data from the OptiTrack motion capture system to get pose feedback of the UAV to complete the loop of the feedback controlled system. To launch the teleoperation and circular trajectory nodes, use the follwoing commands:

Teleoperation:
```
roslaunch autopilot teleop.launch
```

Circular trajectory:
```
roslaunch autopilot circle.launch
```

## Contact

For any issues or bugs in the code, feel free to contact me at `mohdfarhanharoon[at]gmail[dot]com`

**Mohd Farhan Haroon,  
Integral Robotics Lab,  
Integral University,  
Lucknow, India - 226026**
