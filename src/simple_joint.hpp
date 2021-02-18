#ifndef SIMPLE_JOINT_H
#define SIMPLE_JOINT_H

#include <kinematic_model/kinematic_model.hpp>

using namespace kinematic_model;

class simple_joint_t
    : public kinematic_model_t
{
public:
    simple_joint_t();

private:
    void build_geometry(geometry::design_t& design) const override;
    void state_transition(const Eigen::VectorXd& xp, const Eigen::VectorXd& q, Eigen::VectorXd& x) const override;
    void observation(const Eigen::VectorXd& x, const Eigen::VectorXd& r, Eigen::VectorXd& z) const override;
};

REGISTER_PLUGIN(simple_joint_t)

#endif