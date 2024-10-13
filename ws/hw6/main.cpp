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
    Problem2D point_problem = HW2::getWorkspace2();
    Problem2D manip_problem = HW6::getHW4Problem2();
    
    // Construct point-agent and manipulator cspace instances.
    std::size_t n_cells = (point_problem.x_max - point_problem.x_min)/0.25;
    std::cout << "n_cells: " << n_cells << "\n";
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctor = std::make_shared<MyPointAgentCSConstructor>(n_cells);
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(n_cells);
    std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();
    
    // Populate the cspace cells with collision values for visulaization.
    std::unique_ptr<amp::GridCSpace2D> point_cspace = point_agent_ctor->construct(point_problem);
    std::unique_ptr<amp::GridCSpace2D> manipulator_cspace = manipulator_ctor->construct(manipulator, manip_problem);

    // Combine your wavefront planner with a cspace object (you do not need to modify these classes).
    PointWaveFrontAlgorithm point_algo(wf_algo, point_agent_ctor);
    ManipulatorWaveFrontAlgorithm manip_algo(wf_algo, manipulator_ctor);

    // Return a path for the point-agent and manipulator using c-space planning.
    Path2D path = point_algo.plan(point_problem);
    //std::cout << "Path length: '"<< path.length() << '\n';
    // Visualizer::makeFigure(point_problem, path); // Visualize path in workspace
    //Visualizer::makeFigure(*point_cspace, path); // Visualize path in cspace

    // ManipulatorTrajectory2Link trajectory = manip_algo.plan(manipulator, manip_problem);
    //Visualizer::makeFigure(manip_problem, manipulator, trajectory);
    //Visualizer::makeFigure(*manipulator_cspace, trajectory);

    //For Exercise 3, you will need to implement the A* algorithm.
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    MyAStarAlgo algo;
    // MyAStarAlgo::GraphSearchResult result = algo.search(problem, heuristic);

    HW6::grade(point_algo, manip_algo, algo, "shaya.naimi@colorado.edu", argc, argv);


    Visualizer::showFigures();
    return 0;

}