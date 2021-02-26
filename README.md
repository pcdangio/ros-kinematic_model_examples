# kinematic_model_examples

This package provides tutorials for using the ROS [kinematic_model](https://github.com/pcdangio/ros-kinematic_model) package. These examples provide sample code that show how to write robot-specific plugins for use with kinematic_model. You can build the examples in this package and run them in kinematic model.

**Examples:**
- [simple_joint](#simple_joint): A simple robot with two links connected by a single revolute joint, with a single sensor measuring joint position.
- [simple_pose](#simple_pose): Estimating a robot's pose in a "world" frame, with an offset position sensor and an orientation sensor.

## Download & Build

To download and build the examples in this package:

```bash
# Switch to your ROS catkin workspace directory
cd my_ros_workspace

# Source ROS environment if you haven't already
source /opt/ros/melodic/setup.bash

# Clone dependencies into src/ if you haven't already
git clone https://github.com/pcdangio/ros-kalman_filter.git src/kalman_filter
git clone https://github.com/pcdangio/ros-transform.git src/transform
git clone https://github.com/pcdangio/ros-transform_msgs.git src/transform_msgs
git clone https://github.com/pcdangio/ros-kinematic_model.git src/kinematic_model

# Clone the kinematic_model package into src/
git clone https://github.com/pcdangio/ros-kinematic_model_examples.git src/kinematic_model_examples

# Build with catkin_make
# NOTE: Compiling as a release with "catkin_make -DCMAKE_BUILD_TYPE=release" gives ~10x improvement in Eigen's computation speed
catkin_make
```

## Example Detail

### simple_joint

This example shows how to create a plugin for a simple robot that has two links connected with a single revolute joint. The robot model in this scenario has a single state variable and a single sensor/observer. The state variable is the joint's angular position, in radians. The sensor is an encoder that directly measures the joint's angular position. The plugin also publishes the estimated joint position each time a new state is calculated.

**Source Code**: [simple_joint.hpp](https://github.com/pcdangio/ros-kinematic_model_examples/blob/main/src/simple_joint.hpp) [simple_joint.cpp](https://github.com/pcdangio/ros-kinematic_model_examples/blob/main/src/simple_joint.cpp)

To run this example, start up a ROS master and then:

```bash
# Set the plugin path parameter. NOTE: Use your compiled path of libsimple_joint_plugin.so
rosparam set /kinematic_model/plugin_path /SOME/PATH/TO/LIBSIMPLE_JOINT_PLUGIN.SO

# Run the kinematic model node.
rosrun kinematic_model node
```

You may now interact with the node in various ways:

```bash
# In one terminal, you can echo the joint state that is being published by the plugin:
rostopic echo /kinematic_model/joint

# In another terminal, you can publish "simulated" sensor data at 10Hz that the plugin will listen to:
rostopic pub /joint_angle std_msgs/Float64 "data: 1.5708" -r 10
# While the sensor data is publishing, you can watch the published joint state on /kinematic_model/joint change.

# In another terminal, you can use the get_transform service to view real-time transforms between the various robot frames:
rosservice call /kinematic_model/get_transform "source_frame: 'link_a'                                  
target_frame: 'link_b'" 
```

### simple_pose

This example shows how to create a plugin for a mobile robot that is moving around in a "world" frame. The robot model in this scenario has 7 state variables, which represent the x/y/z position and the quaternion (qw/qx/qy/qz) orientation of the robot in the "robot" frame. It has two sensors: The first sensor is a position sensor, which measures x/y/z position. The position sensor itself is mounted on the robot, and has a positional offset of (0,-0.2,0.1) between the sensor's frame and the "robot" frame. The second sensor is an orientation sensor, which directly measures the robot's quaternion (qw/qx/qy/qz) orientation in the "robot" frame. These two sensors combined create 3 + 4 = 7 observers for the purpose of state estimation. This plugin also publishes the robot's estimated pose in the "world" frame.

The `observation()` function in this plugin shows how to use the base class's `get_transform()` for implementing the robot's observation model. As mentioned above, the position sensor's measurements are taken in the "position_sensor" frame, which has a fixed offset from the "robot" frame. Since the position state variables are in the "robot" frame, they need to be transformed into the "position_sensor" frame in order to calculate the expected position sensor measurements.

**Source Code**: [simple_pose.hpp](https://github.com/pcdangio/ros-kinematic_model_examples/blob/main/src/simple_pose.hpp) [simple_pose.cpp](https://github.com/pcdangio/ros-kinematic_model_examples/blob/main/src/simple_pose.cpp)

To run this example, start up a ROS master and then:

```bash
# Set the plugin path parameter. NOTE: Use your compiled path of libsimple_pose_plugin.so
rosparam set /kinematic_model/plugin_path /SOME/PATH/TO/LIBSIMPLE_POSE_PLUGIN.SO

# Run the kinematic model node.
rosrun kinematic_model node
```

You may now interact with the node in various ways:

```bash
# In one terminal, you can echo the robot's pose (in the "world" frame) that is being published by the plugin:
rostopic echo /kinematic_model/pose 

# In two other terminals, you can publish "simulated" sensor data at 10Hz that the plugin will listen to.
# Publisher for position data:
rostopic pub /position geometry_msgs/Point "x: 1.0
y: 2.0
z: 3.0" -r 10 
# Publisher for orientation data:
rostopic pub /orientation geometry_msgs/Quaternion "x: 0.0638159
y: -0.073467
z: 0.3773521
w: 0.9209427" -r 10
# While the sensor data is publishing, you can watch the published pose on /kinematic_model/pose change.

# In another terminal, you can use the get_transform service to view real-time transforms between the various robot frames:
rosservice call /kinematic_model/get_transform "source_frame: 'position_sensor'                                  
target_frame: 'world'" 
```