// SEE README.md FOR details on this example's configuration.

#include "simple_pose.hpp"

using namespace kinematic_model;

// CONSTRUCTOR
simple_pose_t::simple_pose_t()
    // Initialze the base class with 7 state variables and 7 sensors
    : kinematic_model_t(7, 7)
{
    // Initialize state estimation covariances.
    simple_pose_t::Q.diagonal().fill(0.0001);
    simple_pose_t::R.diagonal().fill(0.01);

    // Initialize state.
    Eigen::VectorXd x_o(7);
    x_o.setZero();
    Eigen::MatrixXd P_o(7,7);
    P_o.setZero();
    P_o.diagonal().fill(0.5);
    simple_pose_t::initialize_state(x_o, P_o);

    // Set up subscribers for position and orientation sensor messages.
    ros::NodeHandle node;
    simple_pose_t::m_subscriber_position_sensor = node.subscribe<geometry_msgs::Point>("position", 1, &simple_pose_t::callback_position_sensor, this);
    simple_pose_t::m_subscriber_orientation_sensor = node.subscribe<geometry_msgs::Quaternion>("orientation", 1, &simple_pose_t::callback_orientation_sensor, this);

    // Set up publisher for estimated pose messages.
    ros::NodeHandle private_node("~");
    simple_pose_t::m_publisher_pose = private_node.advertise<geometry_msgs::Pose>("pose", 1);
}

// REQUIRED OVERRIDES
void simple_pose_t::build_geometry(geometry::design_t& design) const
{
    // Create a world frame to track the robot's pose in.
    auto frame_world = design.create_frame("world");

    // Create a single base link for the robot.
    auto link_robot = design.create_link("robot");

    // The position sensor is offset from the robot frame's origin.
    // Create a frame for the position sensor.
    auto frame_position_sensor = design.create_frame("position_sensor");

    // Add world frame to the design.
    design.add_object(frame_world);

    // Add the robot link, attaching it to the world frame with a dynamic pose attachment.
    // The state indices correspond to the state vector: x y z qw qx qy qz.
    design.add_object(link_robot, frame_world, 0, 1, 2, 3, 4, 5, 6);

    // Add the position_frame to the robot, with a fixed attachment using the sensor's offset position.
    // Sensor is offset from robot origin by x = 0, y = -0.2, z = 0.1
    design.add_object(frame_position_sensor, link_robot, 0, -0.2, 0.1, 0, 0, 0);
}
void simple_pose_t::state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const
{
    // For basic example, assume the pose doesn't change.
    x = xp;
}
void simple_pose_t::observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const
{
    // State x/y/z are in robot frame. Position sensors z(0)/z(1)/z(2) are in position_sensor frame.
    // Get the transform from the robot frame to the position_sensor frame.
    transform::transform_t transform;
    simple_pose_t::get_transform("robot", "position_sensor", x, transform);
    // Apply transform to the robot's x/y/z state to get the expected position sensor measurement.
    Eigen::Vector3d position;
    position.x() = x(0);
    position.y() = x(1);
    position.z() = x(2);
    transform.transform(position);
    // Set transformed position as the expected position sensor measurement.
    z(0) = position.x();
    z(1) = position.y();
    z(2) = position.z();

    // Orientation sensor directly observes the orientation state.
    z(3) = x(3);
    z(4) = x(4);
    z(5) = x(5);
    z(6) = x(6);
}

// SENSOR SUBSCRIBER CALLBACKS
void simple_pose_t::callback_position_sensor(const geometry_msgs::PointConstPtr& message)
{
    // Add position measurements to base class state estimator.
    // z = x y z qw qx qy qz
    simple_pose_t::new_observation(0, message->x);
    simple_pose_t::new_observation(1, message->y);
    simple_pose_t::new_observation(2, message->z);
}
void simple_pose_t::callback_orientation_sensor(const geometry_msgs::QuaternionConstPtr& message)
{
    // Add quaternion measurements to base class state estimator.
    // z = x y z qw qx qy qz
    simple_pose_t::new_observation(3, message->w);
    simple_pose_t::new_observation(4, message->x);
    simple_pose_t::new_observation(5, message->y);
    simple_pose_t::new_observation(6, message->z);
}

// STATE PUBLISHING
void simple_pose_t::on_state_update()
{
    // Use method to publish the robot's current estimated pose in the world frame.
    // This method is called automatically each time a new state is calculated.
    
    // Get the current state.
    auto& state = simple_pose_t::state();

    // Publish the current estimated pose.
    geometry_msgs::Pose pose_message;
    pose_message.position.x = state(0);
    pose_message.position.y = state(1);
    pose_message.position.z = state(2);
    pose_message.orientation.w = state(3);
    pose_message.orientation.x = state(4);
    pose_message.orientation.y = state(5);
    pose_message.orientation.z = state(6);
    simple_pose_t::m_publisher_pose.publish(pose_message);
}