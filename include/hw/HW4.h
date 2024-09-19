#pragma once

#include <memory>

#include "tools/Environment.h"
#include "tools/Obstacle.h"
#include "tools/LinkManipulator.h"
#include "tools/ConfigurationSpace.h"

namespace amp {

class ManipulatorCSConstructor {
    public:
        /******* User Implemented Methods ********/

        /// @brief Create a configuration space object given a maniplator and an environment.
        /// @param manipulator Two link manipulator (consider ussing `ASSERT` to make sure the manipulator is 2D)
        /// @param env Environment
        /// @return Unique pointer to your constructed C-space object. 
        /// NOTE: We use a unique pointer here to be able to move the C-space without copying it, since grid discretization
        /// C-spaces can contain a LOT of memory, so copying would be a very expensive operation. Additionally, a pointer is polymorphic
        /// which allows the type to pose as a GridCSpace2D (even though GridCSpace2D is abstract)
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) = 0;

        /*****************************************/
    
        virtual ~ManipulatorCSConstructor() {}
};

class HW4 {
    public:
        /// @brief Get triangle obstacle described in Exercise 1
        /// @return Trangle obstacle
        static amp::Obstacle2D getEx1TriangleObstacle();

        /// @brief Get workspace described in Exercise 3(a)
        /// @return Environment with one triangle obstacle
        static amp::Environment2D getEx3Workspace1();

        /// @brief Get workspace described in Exercise 3(b)
        /// @return Environment with two large rectangular obstacles
        static amp::Environment2D getEx3Workspace2();

        /// @brief Get workspace described in Exercise 3(c)
        /// @return Environment with the obstacles from (b) and another rectangle
        static amp::Environment2D getEx3Workspace3();

        /// @brief Check you forward kinematics
        /// @param joint_location Your computed joint location
        /// @param joint_index The index of the joint whose location you computed. `joint_index == nLinks()` represents the end-effector (last) joint
        /// @param manipulator Link manipulator with any number of links
        /// @param state The joint angle state of the manipulator. Length must match number of links
        /// @param verbose Display results in terminal
        /// @return `true` if the joint location is correct, `false` otherwise
        static bool checkFK(const Eigen::Vector2d& joint_location, uint32_t joint_index, const amp::LinkManipulator2D& manipulator, const amp::ManipulatorState& state, bool verbose = true);

        /// @brief Check how well a given C-space matches the true (sampled) cspace
        /// @param cspace Configuration space of any type
        /// @param manipulator Link manipulator with two links
        /// @param env Environment with any number of obstacles
        /// @param n_samples Number of samples approximating true C-space
        /// @param verbose Display results in terminal
        /// @return Percent success in [0.0, 1.0]
        static double checkCSpace(const ConfigurationSpace2D& cspace, const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env, uint32_t n_samples = 10000, bool verbose = true);

        /// @brief Generate a random environment and check you C-space constructor against it
        /// @param cspace_constructor Your C-space constructor
        /// @param manipulator You manipulator
        /// @param n_samples Number of samples approximating true C-space
        /// @param verbose Display results in terminal
        /// @return Percent success in [0.0, 1.0]
        static double generateAndCheck(ManipulatorCSConstructor& cspace_constructor, const amp::LinkManipulator2D& manipulator, uint32_t n_samples = 10000, bool verbose = true);

        /// @brief Generate a random environment and check you C-space constructor against it
        /// @param cspace_constructor Your C-space constructor
        /// @param manipulator You manipulator
        /// @param random_env Random environment that was generated stored in-place
        /// @param n_samples Number of samples approximating true C-space
        /// @param verbose Display results in terminal
        /// @return Percent success in [0.0, 1.0]
        static double generateAndCheck(ManipulatorCSConstructor& cspace_constructor, const amp::LinkManipulator2D& manipulator, amp::Environment2D& random_env, std::unique_ptr<amp::GridCSpace2D>& cspace, uint32_t n_samples = 10000, bool verbose = true);

        /// @brief Grade your cspace constructor using your custom manipulator type
        /// @tparam MANIPULATOR_T Your custom manipulator type. Must be a subclass of amp::LinkManipulator2D. Make sure your class has a default constructor
        /// @param cspace_constructor Your cspace constructor object
        /// @param email Your identikey@colorado.edu email
        /// @param argc Pass the cmd line args from main
        /// @param argv Pass the cmd line args from main
        template <class MANIPULATOR_T>
        static int grade(ManipulatorCSConstructor& cspace_constructor, const std::string& email, int argc, char** argv);

    private:
        static int grade(ManipulatorCSConstructor& cspace_constructor, amp::LinkManipulator2D& manipulator, const std::string& email, int argc, char** argv);
        static void gradeFK(const amp::LinkManipulator2D& manipulator, bool& fk_pass);
        static void gradeIK(const amp::LinkManipulator2D& manipulator, bool& ik_pass);
        static void gradeEx3(ManipulatorCSConstructor& cspace_constructor, const amp::LinkManipulator2D& manipulator, bool& ws1_pass, bool& ws2_pass, bool& ws3_pass);
};

#define AMP_HW4_ALIAS "hw4"
#define AMP_HW4_PACKAGE_NAME "hw4_report_card"
}

#include "public/HW4_impl.h"