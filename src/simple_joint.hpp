#ifndef SIMPLE_JOINT_H
#define SIMPLE_JOINT_H

#include <kinematic_model/kinematic_model.hpp>

#include <std_msgs/Float64.h>

using namespace kinematic_model;

class simple_joint_t
    : public kinematic_model_t
{
public:
    simple_joint_t();

private:
    void build_geometry(geometry::design_t& design) const override;
    void state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const override;
    void observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const override;

    ros::Subscriber m_subscriber_joint_sensor;
    void callback_joint_sensor(const std_msgs::Float64ConstPtr& message);
};

REGISTER_PLUGIN(simple_joint_t)

#endif