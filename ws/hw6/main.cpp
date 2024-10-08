#include "AMPCore.h"

#include "hw/HW2.h"
#include "hw/HW6.h"

#include "MyAStar.h"
#include "MyCSConstructors.h"
#include "ManipulatorSkeleton.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // You will need your 2-link manipulator from HW4
    MyManipulator2D manipulator;
    Problem2D point_problem = HW2::getWorkspace1();
    Problem2D manip_problem = HW6::getHW4Problem2();
    
    // Construct point-agent and manipulator cspace instances.
    std::size_t n_cells = 5;
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctor = std::make_shared<MyPointAgentCSConstructor>(n_cells);
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(n_cells);
    std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();
    
    // Combine your wavefront planner with a cspace object (you do not need to modify these classes).
    PointWaveFrontAlgorithm point_algo(wf_algo, point_agent_ctor);
    ManipulatorWaveFrontAlgorithm manip_algo(wf_algo, manipulator_ctor);

    // Return a path for the point-agent and manipulator using c-space planning.
    Path2D path = point_algo.plan(point_problem);
    Visualizer::makeFigure(point_problem, path); // Visualize path in workspace
    Visualizer::makeFigure(*point_algo.getCSpace(), path); // Visualize path in cspace

    ManipulatorTrajectory2Link trajectory = manip_algo.plan(manipulator, manip_problem);
    Visualizer::makeFigure(manip_problem, manipulator, trajectory);
    Visualizer::makeFigure(*manip_algo.getCSpace(), trajectory);

    // For Exercise 3, you will need to implement the A* algorithm.
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(problem, heuristic);

    Visualizer::showFigures();

    amp::HW6::grade<PointWaveFrontAlgorithm, ManipulatorWaveFrontAlgorithm, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(wf_algo, point_agent_ctor), std::make_tuple(wf_algo, manipulator_ctor), std::make_tuple());
    return 0;
}