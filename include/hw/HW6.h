#pragma once

#include <memory>

#include "tools/Environment.h"
#include "tools/Obstacle.h"
#include "tools/LinkManipulator.h"
#include "tools/ConfigurationSpace.h"
#include "tools/Algorithms.h"

#include "hw/HW4.h"

namespace amp {

/// @brief Base class for implementing the WaveFront algorithm
class WaveFrontAlgorithm {
    public:
        /******* User Implemented Methods ********/

        /// @brief Return a non-colliding path through a grid C-space using the WaveFront algorithm. Override this method and implement your WaveFront planner
        /// @param grid_cspace Your grid discretization C-space from HW4. 
        /// NOTE: For Exercise 1), you will need to manually construct the discretization for HW2 Exercise2.
        /// NOTE: For Exercise 2), You can use 
        /// @return A path inside the C-space. Your WaveFront planner will find a sequence of cells. This path should be a sequence of representative points for each cell in the sequence.
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) = 0;

        /*****************************************/
};

/// @brief WaveFront for a point agent (Exercise 1)
class PointWaveFrontAlgorithm : public WaveFrontAlgorithm, public PointMotionPlanner2D {
    public:
        /******* User Implemented Methods ********/

        /// @brief Create a discretized planning space of an environment (point agent). If you abstracted your GridCSpace2DConstructor, you may be
        /// able to use that code here to construct a discretized C-space for a point agent.
        /// @param environment Workspace and/or C-space (point agent)
        /// @return Unique pointer to a GridCSpace2D object (see HW4)
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) = 0;

        /*****************************************/

        /// @brief For this problem, I have implemented the plan method for you! This method uses the tools you have created to setup the WaveFront problem.
        /// Make sure you understand the flow of this implementation. You do NOT need to edit anything in this function, or override it in your class.
        /// @param link_manipulator_agent Your link manipulator with forward and inverse kinematics implemented in HW4
        /// @param problem A planning problem with workspace obstacles and init/goal end effector locations
        /// @return A sequence of ManipulatorStates that takes the manipulator from the q_init location to q_goal
        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            // Construct the grid discretized workspace/cspace
            std::unique_ptr<amp::GridCSpace2D> grid_cspace = constructDiscretizedWorkspace(problem);

            // Call method to plan in C-space using the WaveFront algorithm using the discretized workspace
            return planInCSpace(problem.q_init, problem.q_goal, *grid_cspace);
        }

};

/// @brief WaveFront algorithm for manipulation planning problems (Exercise 2)
class ManipulatorWaveFrontAlgorithm : public WaveFrontAlgorithm, public LinkManipulatorMotionPlanner2D {
    public:
        /// @brief Construct the algorithm class by providing a C-space constructor member (from HW4). 
        /// @param c_space_constructor Shared pointer to a C-space constructor object. 
        ManipulatorWaveFrontAlgorithm(const std::shared_ptr<GridCSpace2DConstructor>& c_space_constructor) : m_c_space_constructor(c_space_constructor) {}

        /// @brief For this problem, I have implemented the plan method for you! This method uses the tools you have created to setup the WaveFront problem.
        /// Make sure you understand the flow of this implementation. You do NOT need to edit anything in this function, or override it in your class.
        /// @param link_manipulator_agent Your link manipulator with forward and inverse kinematics implemented in HW4
        /// @param problem A planning problem with workspace obstacles and init/goal end effector locations
        /// @return A sequence of ManipulatorStates that takes the manipulator from the q_init location to q_goal
        virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
            ASSERT(link_manipulator_agent.nLinks() == 2, "Manipulator must have two links");

            // Get the initial state from IK
            amp::ManipulatorState init_state = link_manipulator_agent.getConfigurationFromIK(problem.q_init);

            // Get the goal state from IK
            amp::ManipulatorState goal_state = link_manipulator_agent.getConfigurationFromIK(problem.q_goal);

            // Construct the grid cspace
            std::unique_ptr<amp::GridCSpace2D> grid_cspace = m_c_space_constructor->construct(link_manipulator_agent, problem);

            // Now that we have everything, we can call method to plan in C-space using the WaveFront algorithm
            // Note, we can use the `convert` overloads to easily go between ManipulatorState and ManipulatorState2Link
            return planInCSpace(convert(init_state), convert(goal_state), *grid_cspace);
        }

    protected:
        /// @brief A shared pointer to a GridCSpace2DConstructor object
        std::shared_ptr<GridCSpace2DConstructor> m_c_space_constructor;
};

class HW6 {
    public:
        /// @brief Checks the path generated by the WaveFront algorithm for a point agent against the problem. (Exercise 1)
        /// @param path Path generated by WaveFront algorithm
        /// @param prob Problem that path was generated on
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool checkPointAgentPlan(const amp::Path2D& path, const amp::Problem2D& prob, bool verbose = true);

        /// @brief Checks the path generated by the WaveFront algorithm for a point agent against the problem. (Exercise 1)
        /// Fills collision points for visualization. 
        /// @param path Path generated by WaveFront algorithm
        /// @param prob Problem that path was generated on
        /// @param collision_points Gather collision points found along the path
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool checkPointAgentPlan(const amp::Path2D& path, const amp::Problem2D& prob, std::vector<Eigen::Vector2d>& collision_points, bool verbose = true);

        /// @brief Checks the path generated by the WaveFront algorithm for a manipulator against the problem. (Exercise 1)
        /// @param trajectory Path generated by WaveFront algorithm
        /// @param link_manipulator_agent Manipulator that the trajectory was planned for 
        /// @param prob Problem that path was generated on
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool checkLinkManipulatorPlan(const amp::ManipulatorTrajectory2Link& trajectory, const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& prob, bool verbose = true);

        /// @brief Generates a random problem for a POINT AGENT, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your implemented WaveFront algorithm
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(PointWaveFrontAlgorithm& algo, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random problem for a POINT AGENT, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your implemented WaveFront algorithm
        /// @param path Return the path generated by your algorithm
        /// @param prob Return the randomly generated problem used
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(PointWaveFrontAlgorithm& algo, amp::Path2D& path, amp::Problem2D& prob, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random problem for a MANIPULATOR, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do.
        /// @param algo Your implemented WaveFront algorithm
        /// @param verbose Output logs displaying result
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(ManipulatorWaveFrontAlgorithm& algo, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random problem for a MANIPULATOR, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your implemented WaveFront algorithm
        /// @param trajectory Return the trajectory generated by your algorithm
        /// @param prob Return the randomly generated problem used
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(ManipulatorWaveFrontAlgorithm& algo, amp::ManipulatorTrajectory2Link& trajectory, amp::Problem2D& prob, bool verbose = true, uint32_t seed = 0u);

        // TODO
        //template <class MANIPULATOR_T>
        //static int grade(GridCSpace2DConstructor& cspace_constructor, const std::string& email, int argc, char** argv);

    private:
};

#define AMP_HW6_ALIAS "hw6"
#define AMP_HW6_PACKAGE_NAME "hw6_report_card"
}

//#include "public/HW6_impl.h"