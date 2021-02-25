// This is an example of a minimum kinematic_model plugin.

// Include the base kinematic_model class.
#include <kinematic_model/kinematic_model.hpp>

// Create a class that implements your plugin.
class my_model_t
    // Derive your class from the kinematic_model_t base class.
    : public kinematic_model::kinematic_model_t
{
public:
    // A default constructor is required.
    my_model_t()
        // Initialize base kinematic_model_t class.
        // This example uses 2 state variables and 3 sensors/observers.
        : kinematic_model_t(2,3)
    {
        // Initialize your plugin here.
    }

    // Override this base class method to specify your geometry.
    void build_geometry(kinematic_model::geometry::design_t& design) const override
    {
        // Add your robot's geometry to the "design" reference parameter.
    }

    // Override this base class method to implement your robot's state transition model.
    void state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const override
    {
        // Use the prior state, xp, as well as other inputs from your class, to calculate
        // the new state, x.
    }

    // Override this base class method to implement your robot's observation model.
    void observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const override
    {
        // Use the current state, x, to calculate the expected observations, z.
    }
};

// Register your plugin class so that kinematic_model's node can load it.
REGISTER_PLUGIN(my_model_t)