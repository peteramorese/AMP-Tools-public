#pragma once

#include <memory>

#include "tools/Environment.h"
#include "tools/Obstacle.h"
#include "tools/LinkManipulator.h"
#include "tools/ConfigurationSpace.h"
#include "tools/Graph.h"
#include "tools/Algorithms.h"

#include "hw/HW4.h"

namespace amp {

class PointAgentCSConstructor {
    public:
        /******* User Implemented Methods ********/

        /// @brief Create a configuration space object given a maniplator and an environment. Same thing as in HW4 but using a point-agent collision checker instead of manipulator collision checker
        /// @param manipulator Two link manipulator (consider ussing `ASSERT` to make sure the manipulator is 2D)
        /// @param env Environment
        /// @return Unique pointer to your constructed C-space object. 
        /// NOTE: We use a unique pointer here to be able to move the C-space without copying it, since grid discretization
        /// C-spaces can contain a LOT of memory, so copying would be a very expensive operation. Additionally, a pointer is polymorphic
        /// which allows the type to pose as a GridCSpace2D (even though GridCSpace2D is abstract)
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env) = 0;
    
        /*****************************************/

        virtual ~PointAgentCSConstructor() {}
};

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
    
        virtual ~WaveFrontAlgorithm() {}
};

/// @brief WaveFront for a point agent (Exercise 1). You do NOT need to derive this class, it is already finished for you
class PointWaveFrontAlgorithm : public PointMotionPlanner2D {
    public:
        PointWaveFrontAlgorithm(const std::shared_ptr<WaveFrontAlgorithm>& wf_algo, const std::shared_ptr<PointAgentCSConstructor>& cspace_constructor)
            : m_wf_algo(wf_algo) 
            , m_cspace_constructor(cspace_constructor)
        {}

        /// @brief For this problem, I have implemented the plan method for you! This method uses the tools you have created to setup the WaveFront problem.
        /// Make sure you understand the flow of this implementation. You do NOT need to edit or override anything in this function.
        /// @param problem A planning problem with workspace obstacles and init/goal locations
        /// @return A path that takes the agent from the q_init location to q_goal
        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            // Construct the grid discretized workspace/cspace
            std::unique_ptr<amp::GridCSpace2D> grid_cspace = m_cspace_constructor->construct(problem);

            // Call method to plan in C-space using the WaveFront algorithm using the discretized workspace
            return m_wf_algo->planInCSpace(problem.q_init, problem.q_goal, *grid_cspace);
        }

        virtual ~PointWaveFrontAlgorithm() {}
    
    private:
        /// @brief A shared pointer to the wavefront algorithm object
        std::shared_ptr<WaveFrontAlgorithm> m_wf_algo;
        /// @brief A shared pointer to a PointAgentCSConstructor object
        std::shared_ptr<PointAgentCSConstructor> m_cspace_constructor;
};

/// @brief WaveFront algorithm for manipulation planning problems (Exercise 2). You do NOT need to derive this class, it is already finished for you
class ManipulatorWaveFrontAlgorithm : public LinkManipulatorMotionPlanner2D {
    public:
        /// @brief Construct the algorithm class by providing a C-space constructor member (from HW4). 
        /// @param cspace_constructor Shared pointer to a C-space constructor object. 
        ManipulatorWaveFrontAlgorithm(const std::shared_ptr<WaveFrontAlgorithm>& wf_algo, const std::shared_ptr<ManipulatorCSConstructor>& c_space_constructor) 
            : m_wf_algo(wf_algo)
            , m_cspace_constructor(c_space_constructor) 
        {}

        /// @brief For this problem, I have implemented the plan method for you! This method uses the tools you have created to setup the WaveFront problem.
        /// Make sure you understand the flow of this implementation. You do NOT need to edit or override anything in this function.
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
            std::unique_ptr<amp::GridCSpace2D> grid_cspace = m_cspace_constructor->construct(link_manipulator_agent, problem);

            // Now that we have everything, we can call method to plan in C-space using the WaveFront algorithm
            // Note, we can use the `convert` overloads to easily go between ManipulatorState and ManipulatorState2Link
            return m_wf_algo->planInCSpace(convert(init_state), convert(goal_state), *grid_cspace);
        }

        virtual ~ManipulatorWaveFrontAlgorithm() {}

    protected:
        /// @brief A shared pointer to the wavefront algorithm object
        std::shared_ptr<WaveFrontAlgorithm> m_wf_algo;
        /// @brief A shared pointer to a ManipulatorCSConstructor object
        std::shared_ptr<ManipulatorCSConstructor> m_cspace_constructor;
};

struct LookupSearchHeuristic : public SearchHeuristic {
	/// @brief Get the heuristic value stored in `heuristic_values`. 
	/// @param node Node to get the heuristic value h(node) for. 
	/// @return Heuristic value
	virtual double operator()(amp::Node node) const override {return heuristic_values.at(node);}

    /// @brief Store the heursitic values for each node in a map
    std::map<amp::Node, double> heuristic_values; 
};

class HW6 {
    public:
        /// @brief Get a problem with the same environment as HW4 Exercise 3 Workspace 1
        /// @return Problem with start and goal end-effector positions
        static amp::Problem2D getHW4Problem1();

        /// @brief Get a problem with the same environment as HW4 Exercise 3 Workspace 2
        /// @return Problem with start and goal end-effector positions
        static amp::Problem2D getHW4Problem2();

        /// @brief Get a problem with the same environment as HW4 Exercise 3 Workspace 3
        /// @return Problem with start and goal end-effector positions
        static amp::Problem2D getHW4Problem3();

        /// @brief Get the shortest path graph search problem described in Exercise 3
        /// @return Graph shown in Figure 1, with initial and final states described in pt (a)
        static amp::ShortestPathProblem getEx3SPP();

        /// @brief Get the heuristic described in the caption of Figure 1 (Exercise 3)
        /// @return LookupSearchHeuristic function with the stored values
        static amp::LookupSearchHeuristic getEx3Heuristic();

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

        /// @brief Checks the path generated by the WaveFront algorithm for a manipulator against the problem. (Exercise 2)
        /// @param trajectory Path generated by WaveFront algorithm
        /// @param link_manipulator_agent Manipulator that the trajectory was planned for 
        /// @param prob Problem that path was generated on
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool checkLinkManipulatorPlan(const amp::ManipulatorTrajectory2Link& trajectory, const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& prob, bool verbose = true);

        /// @brief Checks a graph search result for the shortest path problem (Exercise 3)
        /// @param result Graph search result object
        /// @param problem Problem the result was generated on
        /// @param heuristic The heuristic you used (defaults to the zero heuristic). This is only necessary if you want
        /// to test your algorithm against possibly non-admissible heuristics
        /// @param verbose Output logs dislaying result
        /// @return `true` if result is a valid solution and the path cost is correct, `false` otherwise
        static bool checkGraphSearchResult(const amp::AStar::GraphSearchResult& result, const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic = amp::SearchHeuristic(), bool verbose = true);

        /// @brief Generates a random problem for a POINT AGENT, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your implemented WaveFront algorithm (must derive PointWaveFrontAlgorithm)
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(amp::PointMotionPlanner2D& algo, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random problem for a POINT AGENT, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your implemented WaveFront algorithm (must derive PointWaveFrontAlgorithm)
        /// @param path Return the path generated by your algorithm
        /// @param prob Return the randomly generated problem used
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @param collision_points Return collision points (second overload)
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(amp::PointMotionPlanner2D& algo, amp::Path2D& path, amp::Problem2D& prob, bool verbose = true, uint32_t seed = 0u);
        static bool generateAndCheck(amp::PointMotionPlanner2D& algo, amp::Path2D& path, amp::Problem2D& prob, std::vector<Eigen::Vector2d>& collision_points, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random problem for a MANIPULATOR, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do.
        /// @param algo Your implemented WaveFront algorithm (must derive ManipulatorWaveFrontAlgorithm)
        /// @param verbose Output logs displaying result
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(amp::LinkManipulatorMotionPlanner2D& algo, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random problem for a MANIPULATOR, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your implemented WaveFront algorithm (must derive ManipulatorWaveFrontAlgorithm)
        /// @param link_manipulator_agent Edit a link manipulator to have the random link lengths used to solve the problem
        /// @param trajectory Return the trajectory generated by your algorithm
        /// @param prob Return the randomly generated problem used
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @param collision_points Return collision states (second overload)
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(amp::LinkManipulatorMotionPlanner2D& algo, amp::LinkManipulator2D& link_manipulator_agent, amp::ManipulatorTrajectory2Link& trajectory, amp::Problem2D& prob, bool verbose = true, uint32_t seed = 0u);
        static bool generateAndCheck(amp::LinkManipulatorMotionPlanner2D& algo, amp::LinkManipulator2D& link_manipulator_agent, amp::ManipulatorTrajectory2Link& trajectory, amp::Problem2D& prob, std::vector<amp::ManipulatorState2Link>& collision_states, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random shortest path graph search problem, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your implemented AStar algorithm
        /// @param seed Seed the random generator. If the seed is `0u`, no seed is used (random)
        /// @param verbose Output logs displaying result
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(amp::AStar& algo, bool verbose = true, uint32_t seed = 0u);

        /// @brief Generates a random shortest path graph search problem, runs the algorithm, then check the validity of the returned solution.
        /// Similar to what the benchmarker in `grade()` will do
        /// @param algo Your implemented AStar algorithm
        /// @param result Return the result generated by your algorithm
        /// @param prob Return the randomly generated SPP used
        /// @return `true` if path is a valid solution, `false` otherwise
        static bool generateAndCheck(amp::AStar& algo, amp::AStar::GraphSearchResult& result, amp::ShortestPathProblem& problem, bool verbose = true, uint32_t seed = 0u);

        /// @brief Tests your point-agent WF algorithm on HW2 (Workspace1 and Workspace2), manipulator WF algorithm on HW4 (Workspace1, Workspace2, Workspace3),
        /// andj HW2 Workspace2.
        /// NOTE: Make sure your member variables are correctly reset after each call to plan(). This method does not reconstruct the 
        /// GDAlgorithm object that you pass in
        /// @param point_wf_algo Your implemented point-agent WaveFront algorithm (must derive PointWaveFrontAlgorithm)
        /// @param manipulator_wf_algo Your implemented manipulator WaveFront algorithm (must derive ManipulatorWaveFrontAlgorithm)
        /// @param astar_algo Your implemented A* WaveFront algorithm 
        /// @param email Your identikey@colorado.edu email
        /// @param argc Pass the cmd line args from main
        /// @param argv Pass the cmd line args from main
        static int grade(amp::PointMotionPlanner2D& point_wf_algo, amp::LinkManipulatorMotionPlanner2D& manipulator_wf_algo, amp::AStar& astar_algo, const std::string& email, int argc, char** argv);

        /// @brief Tests your point-agent WF algorithm on HW2 (Workspace1 and Workspace2), manipulator WF algorithm on HW4 (Workspace1, Workspace2, Workspace3),
        /// andj HW2 Workspace2.
        /// NOTE: Reconstructs each algorithm object every trial to make sure your member variables are reset, etc...
        /// NOTE: Each algorithm MUST have a default constructor to use this `grade()` overload
        /// @tparam POINT_WF_ALG_T The type of your point-agent WaveFront algorithm object (MUST derive PointWaveFrontAlgorithm)
        /// @tparam ASTAR_ALG_T The type of your manipulator WaveFront algorithm object (MUST derive ManipulatorWaveFrontAlgorithm)
        /// @tparam MANIP_WF_ALG_T The type of your A* algorithm object (MUST derive amp::AStar)
        /// @param email Your identikey@colorado.edu email
        /// @param argc Pass the cmd line args from main
        /// @param argv Pass the cmd line args from main
        template <class POINT_WF_ALG_T, class MANIP_WF_ALG_T, class ASTAR_ALG_T>
        static int grade(const std::string& email, int argc, char** argv);

        /// @brief Tests your point-agent WF algorithm on HW2 (Workspace1 and Workspace2), manipulator WF algorithm on HW4 (Workspace1, Workspace2, Workspace3),
        /// andj HW2 Workspace2.
        /// NOTE: Reconstructs each algorithm object every trial to make sure your member variables are reset, etc...
        /// NOTE: You can pass a std::tuple that contains the constructor parameters for each algorithm. Sorry for the syntax nightmare :(
        /// @tparam POINT_WF_ALG_T The type of your point-agent WaveFront algorithm object (MUST derive PointWaveFrontAlgorithm)
        /// @tparam ASTAR_ALG_T The type of your manipulator WaveFront algorithm object (MUST derive ManipulatorWaveFrontAlgorithm)
        /// @tparam MANIP_WF_ALG_T The type of your A* algorithm object (MUST derive amp::AStar)
        /// @tparam _POINT_CTOR_ARGS_TUP [automatically deduced] Must be a std::tuple (see below)
        /// @tparam _MANIP_CTOR_ARGS_TUP [automatically deduced] Must be a std::tuple (see below)
        /// @tparam _ASTAR_CTOR_ARGS_TUP [automatically deduced] Must be a std::tuple (see below)
        /// @param email Your identikey@colorado.edu email
        /// @param argc Pass the cmd line args from main
        /// @param argv Pass the cmd line args from main
        /// @param point_wf_ctor_args_tuple A std::tuple of your constructor arguments for the point-agent WaveFront algorithm object, 
        /// for example: std::make_tuple(ctor_arg1, ctor_arg2, ...)
        /// @param manip_wf_ctor_args_tuple A std::tuple of your constructor arguments for the manipulator WaveFront algorithm object, 
        /// for example: std::make_tuple(ctor_arg1, ctor_arg2, ...)
        /// @param astar_ctor_args_tuple A std::tuple of your constructor arguments for the A* algorithm object, 
        /// for example: std::make_tuple(ctor_arg1, ctor_arg2, ...)
        template <class POINT_WF_ALG_T, class MANIP_WF_ALG_T, class ASTAR_ALG_T, class _POINT_CTOR_ARGS_TUP, class _MANIP_CTOR_ARGS_TUP, class _ASTAR_CTOR_ARGS_TUP>
        static int grade(const std::string& email, int argc, char** argv, 
                    const _POINT_CTOR_ARGS_TUP& point_wf_ctor_args_tuple,
                    const _MANIP_CTOR_ARGS_TUP& manip_wf_ctor_args_tuple,
                    const _ASTAR_CTOR_ARGS_TUP& astar_ctor_args_tuple);
        /* 
        Example syntax usage of this method:

        Description:    Algo type      Algo type      Algo type                                                 No ctor params     Pass a string param       Pass multiple params
                   :                                                                                            for MyPointWFAlgo  to MyManipWFAlgoPass      to MyAStarAlgo
        amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("alien"), std::make_tuple("borp", 5, 10.0));

        */

    private:
        static void assertDerivesPoint(amp::PointMotionPlanner2D& algo);
        static void assertDerivesManipulator(amp::LinkManipulatorMotionPlanner2D& algo);
};

#define AMP_HW6_ALIAS "hw6"
#define AMP_HW6_PACKAGE_NAME "hw6_report_card"
}

#include "public/HW6_impl.h"