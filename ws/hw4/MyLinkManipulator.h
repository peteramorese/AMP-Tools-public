#include "AMPCore.h"

using namespace amp;

class MyLinkManipulator : public LinkManipulator2D {
    public:
        MyLinkManipulator();
        MyLinkManipulator(const std::vector<double>& link_lengths);
        MyLinkManipulator(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths);

        virtual Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override;
        virtual ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;

        // private:
        //      linkLengths;
};