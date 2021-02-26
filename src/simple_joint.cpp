// SEE README.md FOR details on this example's configuration.

#include "simple_joint.hpp"

using namespace kinematic_model;

// CONSTRUCTOR
simple_joint_t::simple_joint_t()
    // Initialze the base class with 1 state variable (joint position) and 1 sensor
    : kinematic_model_t(1, 1)
{
    // Initialize state estimation covariances.
    simple_joint_t::Q(0,0) = 0.0001;
    simple_joint_t::R(0,0) = 0.01;

    // Initialize state.
    Eigen::VectorXd x_o(1);
    Eigen::MatrixXd P_o(1,1);
    x_o(0) = 0;
    P_o(0,0) = 0.5;
    simple_joint_t::initialize_state(x_o, P_o);

    // Set up subscriber for joint angle messages.
    ros::NodeHandle node;
    simple_joint_t::m_subscriber_joint_sensor = node.subscribe<std_msgs::Float64>("joint_angle", 1, &simple_joint_t::callback_joint_sensor, this);

    // Set up publisher for estimated joint angle messages.
    ros::NodeHandle private_node("~");
    simple_joint_t::m_publisher_joint = private_node.advertise<std_msgs::Float64>("joint", 1);
}

// REQUIRED OVERRIDES
void simple_joint_t::build_geometry(geometry::design_t& design) const
{
    // Create links.
    auto link_a = design.create_link("link_a");
    auto link_b = design.create_link("link_b");

    // Create joint.
    auto joint_ab = design.create_joint("joint_ab", geometry::object::joint_t::type_t::REVOLUTE, 0);
    // Joint axis defaults to (0,0,1)

    // Add objects to the model's design.
    design.add_object(link_a);
    design.add_object(joint_ab, link_a, 1, 0, 0, 0, 0, 0);
    design.add_object(link_b, joint_ab, 1, 0, 0, 0, 0, 0);
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
    
    // Get the current state.
    auto& state = simple_joint_t::state();

    // Publish the joint's current estimated position.
    std_msgs::Float64 joint_message;
    joint_message.data = state(0);
    simple_joint_t::m_publisher_joint.publish(joint_message);
}

// SENSOR SUBSCRIBER CALLBACK
void simple_joint_t::callback_joint_sensor(const std_msgs::Float64ConstPtr& message)
{
    // Add sensor measurement as new observation.
    simple_joint_t::new_observation(0, message->data);
}