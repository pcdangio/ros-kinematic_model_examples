# kinematic_model_examples

This package provides tutorials for using the ROS [kinematic_model](https://github.com/pcdangio/ros-kinematic_model) package. These examples provide sample code that show how to write robot-specific plugins for use with kinematic_model. You can build the examples in this package and run them in kinematic model.

**Examples:**
- [simple_joint](#simple_joint): A simple robot with two links connected by a single revolute joint, with a single sensor measuring joint position.
- [simple_pose](#simple_pose): Estimating a robot's pose in a "world" frame, with a position sensor and an orientation sensor.

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

This example shows how to create a plugin for a simple robot that has two links connected with a single revolute joint. The example uses a single sensor that measures the joint's position. The example also publishes the estimated joint position each time a new state is calculated.

Take a look at the example's source code to understand what's happening: [simple_joint.hpp](https://github.com/pcdangio/ros-kinematic_model_examples/blob/main/src/simple_joint.hpp) & [simple_joint.cpp](https://github.com/pcdangio/ros-kinematic_model_examples/blob/main/src/simple_joint.cpp)

To run this example, start up a ROS master and then:

```bash
# Set the plugin path parameter. NOTE: Use your compiled path of libsimple_joint_plugin.so
rosparam set /kinematic_model/plugin_path /SOME/PATH/TO/LIBSIMPLE_JOINT_PLUGIN.SO

# Run the kinematic model node.
rosrun kinematic_model node
```

You may now interact with the node in various ways:


### simple_pose
