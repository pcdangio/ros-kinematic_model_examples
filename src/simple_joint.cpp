#include "simple_joint.hpp"

using namespace kinematic_model;

simple_joint_t::simple_joint_t()
    : kinematic_model_t(1, 1)
{
    // Initialize state estimation covariances.
    simple_joint_t::Q(0,0) = 0.0001;
    simple_joint_t::R(0,0) = 0.01;

    // Initialize state.
    Eigen::VectorXd x_o(1);
    Eigen::VectorXd P_o(1,1);
    x_o(0) = 0;
    P_o(0,0) = 0.01;
    simple_joint_t::initialize_state(x_o, P_o);

    // Set up subscriber for joint angle messages.
    ros::NodeHandle node;
    simple_joint_t::m_subscriber_joint_sensor = node.subscribe<std_msgs::Float64>("joint_angle", 1, &simple_joint_t::callback_joint_sensor, this);
}

void simple_joint_t::build_geometry(geometry::design_t& design) const
{
    // Create geometry objects.
    auto link_a = design.create_link("link_a");
    auto link_b = design.create_link("link_b");
    auto joint_ab = design.create_joint("joint_ab", geometry::object::joint_t::type_t::REVOLUTE, 0);

    // Add objects to the model's design.
    design.add_object(link_a);
    design.add_object(link_b, link_a, joint_ab);
}
void simple_joint_t::state_transition(const Eigen::VectorXd& xp, const Eigen::VectorXd& q, Eigen::VectorXd& x) const
{
    // For basic example, assume the joint position doesn't change.
    // Assume additive process noise.
    x(0) = xp(0) + q(0);
}
void simple_joint_t::observation(const Eigen::VectorXd& x, const Eigen::VectorXd& r, Eigen::VectorXd& z) const
{
    // For basic example, observer 0 is a direct observer of state 0.
    // Assume additive measurement noise.
    z(0) = x(0) + r(0);
}

void simple_joint_t::callback_joint_sensor(const std_msgs::Float64ConstPtr& message)
{
    // Add sensor measurement as new observation.
    simple_joint_t::new_observation(0, message->data);
}