// SEE README.md FOR details on this example's configuration.

#include "simple_joint.hpp"

using namespace kinematic_model;

// CONSTRUCTOR
simple_joint_t::simple_joint_t()
    // Initialze the base class with 1 state variable (joint position) and 1 sensor
    : kinematic_model_t(1, 1)
{
    // Initialize process and measurement covariances.
    simple_joint_t::Q(0,0) = 0.0001;
    simple_joint_t::R(0,0) = 0.01;

    // Initialize state.
    simple_joint_t::set_state(0, 0);
    // Initialize covariance.
    simple_joint_t::set_covariance(0, 0, 0.5);

    // Set up subscriber for joint angle messages.
    ros::NodeHandle node;
    simple_joint_t::m_subscriber_joint_sensor = node.subscribe<std_msgs::Float64>("sensed_joint_angle", 1, &simple_joint_t::callback_joint_sensor, this);

    // Set up publisher for estimated joint angle messages.
    ros::NodeHandle private_node("~");
    simple_joint_t::m_publisher_joint = private_node.advertise<std_msgs::Float64>("estimated_joint_angle", 1);
}

// REQUIRED OVERRIDES
void simple_joint_t::build_geometry(geometry::design_t& design) const
{
    // Create link objects.
    auto link_a = geometry::object::link_t::create("link_a");
    auto link_b = geometry::object::link_t::create("link_b");

    // Create revolute joint with state variable 0 as the joint angle.
    auto joint_ab = geometry::object::joint_t::create("joint_ab", geometry::object::joint_t::type_t::REVOLUTE, 0);
    // Joint axis defaults to (0,0,1)

    // Add objects to the model's design.
    // Add link_a as a base/root object.
    design.add_object(link_a);
    // Add joint_ab as a child to link_a, with a fixed translation / fixed rotation (FTFR) attachment.
    design.add_object(joint_ab, link_a, geometry::attachment::ftfr_t::create(1, 0, 0, 0, 0, 0));
    // Add link_b as a child to joint_ab, with a fixed translation / fixed rotation (FTFR) attachment.
    design.add_object(link_b, joint_ab, geometry::attachment::ftfr_t::create(1, 0, 0, 0, 0, 0));
}
void simple_joint_t::state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const
{
    // For basic example, assume the joint position doesn't change.
    x(0) = xp(0);
}
void simple_joint_t::observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const
{
    // For basic example, observer 0 is a direct observer of state 0.
    z(0) = x(0);
}

// STATE PUBLISHING
void simple_joint_t::on_state_update()
{
    // Use method to publish the joint's current state.
    // This method is called automatically each time a new state is calculated.

    // Publish the joint's current estimated position.
    std_msgs::Float64 joint_message;
    joint_message.data = simple_joint_t::state(0);
    simple_joint_t::m_publisher_joint.publish(joint_message);
}

// SENSOR SUBSCRIBER CALLBACK
void simple_joint_t::callback_joint_sensor(const std_msgs::Float64ConstPtr& message)
{
    // Add sensor measurement as new observation.
    simple_joint_t::new_observation(0, message->data);
}