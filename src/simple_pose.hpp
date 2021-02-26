// SEE README.md FOR details on this example's configuration.

#ifndef SIMPLE_POSE_H
#define SIMPLE_POSE_H

// Include the kinematic_model base class.
#include <kinematic_model/kinematic_model.hpp>

// Used to subscribe to pose position/orientation sensors.
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// Used for publishing our robot's pose state.
#include <geometry_msgs/Pose.h>

using namespace kinematic_model;

// Create a new plugin class.
class simple_pose_t
    // Extend the base kinematic_model class.
    : public kinematic_model_t
{
public:
    // A default constructor is required.
    simple_pose_t();

private:
    //  Override the required base class methods.
    void build_geometry(geometry::design_t& design) const override;
    void state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const override;
    void observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const override;

    // Optionally override the on_state_update() method for publishing the joint state.
    void on_state_update() override;

    // Create a subscriber for the position sensor.
    ros::Subscriber m_subscriber_position_sensor;
    void callback_position_sensor(const geometry_msgs::PointConstPtr& message);

    // Create a subscriber for the orientation sensor.
    ros::Subscriber m_subscriber_orientation_sensor;
    void callback_orientation_sensor(const geometry_msgs::QuaternionConstPtr& message);

    // Create a publisher for sending the robot's estimated pose in the world frame.
    ros::Publisher m_publisher_pose;
};

// Register the custom plugin.
REGISTER_PLUGIN(simple_pose_t)

#endif