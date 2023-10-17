#pragma once

#include <vector>
#include <Eigen/Core>

#include "tools/Path.h"
#include "tools/Serializer.h"

namespace amp {

/// @brief Vector of angles (radians) for each joint. The size of the vector should match the 
/// number of links (and hence joints) of the manipulator
using ManipulatorState = std::vector<double>;
/// @brief For the specific 2-link case, use Eigen::Vector2d to make it consistent with other 2D planning problems
using ManipulatorState2Link = Eigen::Vector2d;

/// @brief List of manipulator states in chronological order
using ManipulatorTrajectory = Path;
/// @brief For the specific 2-link case, use Path2D to make it consistent with other 2D planning problems
using ManipulatorTrajectory2Link = Path2D;


class LinkManipulator2D {
    public:
        /// @brief Construct 2-link manipulator. The base location is set to (0.0, 0.0), each link_length is 1.0
        LinkManipulator2D();

        /// @brief Construct from an array of link lengths. The base location is set to (0.0, 0.0)
        /// @param link_lengths Array of link lengths. The number of link lengths dictates the DOF of the manipulator
        LinkManipulator2D(const std::vector<double>& link_lengths);

        /// @brief Construct from an array of link lengths.
        /// @param base_location Custom base location of the manipulator
        /// @param link_lengths Array of link lengths. The number of link lengths dictates the DOF of the manipulator
        LinkManipulator2D(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths);

        /******* User Implemented Methods ********/

        /// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
        /// @param state Joint angle state (radians). Must have size() == nLinks()
        /// @param joint_index Joint index in order of base to end effector 
        /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
        /// @return Joint coordinate
        virtual Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const = 0;

        /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
        /// @param end_effector_location End effector coordinate
        /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
        virtual ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const = 0;

        /*****************************************/

        /// @brief Get the number of links. The number of links is the same as the number of joints
        /// @return Number of links
        inline std::size_t nLinks() const {return m_link_lengths.size();}

        /// @brief Read the link lengths.
        /// @return Const reference access to link lengths
        inline const std::vector<double>& getLinkLengths() const {return m_link_lengths;}

        /// @brief Edit the link lengths.
        /// @return Reference access to link lengths
        inline std::vector<double>& getLinkLengths() {return m_link_lengths;}

        /// @brief Read the base location.
        /// @return Const reference to base location
        inline const Eigen::Vector2d& getBaseLocation() const {return m_base_location;}

        /// @brief Edit the base location.
        /// @return Reference access to base location
        inline Eigen::Vector2d& getBaseLocation() {return m_base_location;}

        /// @brief Get the maximum radial reach of the manipulator 
        /// @return Sum of all of the link lengths
        double reach() const;

        /// @brief Print the object
        /// @param heading Log what type of object is being printed
        void print(const std::string& heading = "LinkManipulator2D") const;

        /// @brief Virtual dtor
        virtual ~LinkManipulator2D() {}
    protected:
        Eigen::Vector2d m_base_location = Eigen::Vector2d(0.0, 0.0);
        std::vector<double> m_link_lengths;
};

ManipulatorState2Link convert(const ManipulatorState& state_2_link);
ManipulatorState convert(const ManipulatorState2Link& state_2_link);

}