// SEE README.md FOR details on this example's configuration.

#ifndef SIMPLE_JOINT_H
#define SIMPLE_JOINT_H

// Include the kinematic_model base class.
#include <kinematic_model/kinematic_model.hpp>

// Used for publishing our joint's current state.
#include <std_msgs/Float64.h>

using namespace kinematic_model;

// Create a new plugin class.
class simple_joint_t
    // Extend the base kinematic_model class.
    : public kinematic_model_t
{
public:
    // A default constructor is required.
    simple_joint_t();

private:
    //  Override the required base class methods.
    void build_geometry(geometry::design_t& design) const override;
    void state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const override;
    void observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const override;

    // Optionally override the on_state_update() method for publishing the joint state.
    void on_state_update() override;

    // Create a subscriber for the joint's position sensor.
    ros::Subscriber m_subscriber_joint_sensor;
    void callback_joint_sensor(const std_msgs::Float64ConstPtr& message);

    // Create a publisher for sending the joint's estimated state.
    ros::Publisher m_publisher_joint;
};

// Register the custom plugin.
REGISTER_PLUGIN(simple_joint_t)

#endif