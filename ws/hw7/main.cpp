// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
# include "MyPRM.h"
# include "GenericPRM.h"

using namespace amp;

int main(int argc, char** argv) {
    HW7::hint(); // Consider implementing an N-dimensional planner 

    // Example of creating a graph and adding nodes for visualization
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();    
    std::map<amp::Node, Eigen::Vector2d> nodes;
    
    std::vector<Eigen::Vector2d> points = {{3, 3}, {4, 5}, {5, 3}, {6, 5}, {5, 7}, {7, 3}, {8, 5}}; // Points to add to the graph
    for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}, {2, 6, 1}, {3, 6, 1}, {5, 6, 1}}; // Edges to connect
    for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph
    // graphPtr->print();

    // Test PRM on Workspace1 of HW2
    Problem2D problem = HW2::getWorkspace1();
    MyPRM2D prm;
    amp::Path2D path = prm.plan(problem);
    //std::cout << "path length: " << path.length() << "\n";
    //Visualizer::makeFigure(problem, prm.plan(problem), *graphPtr, nodes);
    Visualizer::makeFigure(problem, prm.plan(problem), *prm.get_graph(), prm.get_nodes());

    // Generate a random problem and test RRT
    MyRRT rrt;
    //Path2D path;
    HW7::generateAndCheck(rrt, path, problem);
    // Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    Visualizer::showFigures();

    // Grade method
    //HW7::grade<MyPRM, MyRRT>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}