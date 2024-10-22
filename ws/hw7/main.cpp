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

    Problem2D problem = HW5::getWorkspace1();
    MyPRM2D prm;
    // prm.myN = 250;
    // prm.myr = 2.5;
    // amp::Path2D pathy =  prm.plan(problem);
    // Visualizer::makeFigure(problem, pathy, *prm.get_graph(), prm.get_nodes());
    // std::cout << pathy.length() << std::endl;



    std::list <std::vector<double>> all_computation_times;
    std::list <std::vector<double>> all_path_lengths;
    std::vector<double> all_valid_sols;
    bool bench = false;
    
    if (bench) {
        for (int N : {200, 500, 1000}){
            for (double r: {1, 2}){
            std::cout << "N: " << N << " r: " << r << std::endl;
            
            std::vector<double> path_lengths;
            int valid_sols = 0;
            std::vector<double> computation_times;
            prm.myN = N;
            prm.myr = r;
            for (int i = 0; i < 100; ++i) {
                    // std::cout<< "iteration: " << i << "\n";
                    auto start = std::chrono::high_resolution_clock::now();
                    
                    amp::Path2D path = prm.plan(problem);
                    
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                    computation_times.push_back(duration.count());
                    double plength  = path.length();
                    
                    if (plength > 0) {
                        valid_sols++;
                        path_lengths.push_back(path.length());
                    }


                }
                all_computation_times.push_back(computation_times);
                all_path_lengths.push_back(path_lengths);
                all_valid_sols.push_back(valid_sols);
            
        }
    }
    //const std::list<std::vector<double>> values = all_path_lengths;    
    std::string xlabel = "Hyperparameters n and r";
    std::string ylabel = "Computation Time (microseconds)";
    std::string title = "Smoothing PRM Computation Time (HW2WS2)";
    std::vector<std::string> labels = {"n=200, r=1","n=200, r=2","n=500, r=1", "n=500, r=2", "n=1000, r=1","n=1000, r=2"};
    Visualizer::makeBoxPlot(all_computation_times, labels, 
                               title, xlabel, ylabel);  
    xlabel = "Hyperparameters n and r";
    ylabel = "Path Length";
    title = "Smoothing PRM Path Length (HW2WS2)";
    // labels = {"n=200, r=0.5","n=200, r=1.0","n=200, r=1.5", "n=200, r=2.0", "n=500, r=0.5","n=500, r=1.0","n=500, r=1.5", "n=500, r=2.0"};
    labels = {"n=200, r=1","n=200, r=2","n=500, r=1", "n=500, r=2", "n=1000, r=1","n=1000, r=2"};
    Visualizer::makeBoxPlot(all_path_lengths, labels, 
                               title, xlabel, ylabel); 
    xlabel = "Hyperparameters n and r";
    ylabel = "Number of Valid Solutions out of 100 runs";
    title = "Smoothing PRM Number of Valid Solutions (HW2WS2)";
    // labels = {"n=200, r=0.5","n=200, r=1.0","n=200, r=1.5", "n=200, r=2.0", "n=500, r=0.5","n=500, r=1.0","n=500, r=1.5", "n=500, r=2.0"};
    labels = {"n=200, r=1","n=200, r=2","n=500, r=1", "n=500, r=2", "n=1000, r=1","n=1000, r=2"};
    Visualizer::makeBarGraph(all_valid_sols, labels, 
                               title, xlabel, ylabel); 

    }



    // Test PRM on Workspace1 of HW2
    

    
    
    // std::cout << "completed 100 PRM runs" << "\n";
    // std::cout << "Number of valid solutions: " << valid_sols << "\n";
    // std::cout << "Computation times: \n";
    // for (const auto& time : computation_times) {
    //     std::cout << time << "\n";
    // }
    // std::cout << "Path lengths: \n";
    // for (const auto& length : path_lengths) {
    //     std::cout << length << "\n";
    // }

    // amp::Path2D path = prm.plan(problem);
    //std::cout << "path length: " << path.length() << "\n";
    //Visualizer::makeFigure(problem, prm.plan(problem), *graphPtr, nodes);
    // Visualizer::makeFigure(problem, prm.plan(problem), *prm.get_graph(), prm.get_nodes());

    // Generate a random problem and test RRT
    MyRRT rrt;
    // HW7::generateAndCheck(rrt, path, problem);
    // Visualizer::makeFigure(problem, rrt.plan(problem), *rrt.get_graph(), rrt.get_nodes());


    std::list <std::vector<double>> rrt_all_computation_times;
    std::list <std::vector<double>> rrt_all_path_lengths;
    std::vector<double> rrt_all_valid_sols;
    bool benchrrt = true;
    std::vector<amp::Problem2D> problems = {HW5::getWorkspace1(), HW2::getWorkspace1(), HW2::getWorkspace2()};
    
    if (benchrrt) {
        for (const auto& problem : problems) {
            amp::Path2D pathy = rrt.plan(problem);
            Visualizer::makeFigure(problem, pathy, *rrt.get_graph(), rrt.get_nodes());
            std::cout << pathy.length() << std::endl;
            std::vector<double> path_lengths;
            int valid_sols = 0;
            std::vector<double> computation_times;
            for (int i = 0; i < 100; ++i) {
                    // std::cout<< "iteration: " << i << "\n";
                    auto start = std::chrono::high_resolution_clock::now();
                    
                    amp::Path2D path = rrt.plan(problem);
                    
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                    computation_times.push_back(duration.count());
                    double plength  = path.length();
                    
                    if (plength > 0) {
                        valid_sols++;
                        path_lengths.push_back(path.length());
                    }


                }
                rrt_all_computation_times.push_back(computation_times);
                rrt_all_path_lengths.push_back(path_lengths);
                rrt_all_valid_sols.push_back(valid_sols);
            
        }
           
            

    //const std::list<std::vector<double>> values = all_path_lengths;    
    std::string xlabel = "Workspace";
    std::string ylabel = "Computation Time (microseconds)";
    std::string title = "RRT Computation Time by workspace";
    // std::vector<std::string> labels = {"n=200, r=1","n=200, r=2","n=500, r=1", "n=500, r=2", "n=1000, r=1","n=1000, r=2"};
    std::vector<std::string> labels = {"HW5 WS1", "HW2 WS1", "HW2 WS2"};
    Visualizer::makeBoxPlot(rrt_all_computation_times, labels, 
                               title, xlabel, ylabel);  
    xlabel = "Workspace";
    ylabel = "Path Length";
    title = "RRT Path Length by workspace";
    // labels = {"n=200, r=0.5","n=200, r=1.0","n=200, r=1.5", "n=200, r=2.0", "n=500, r=0.5","n=500, r=1.0","n=500, r=1.5", "n=500, r=2.0"};
    // labels = {"n=200, r=1","n=200, r=2","n=500, r=1", "n=500, r=2", "n=1000, r=1","n=1000, r=2"};
    labels = {"HW5 WS1", "HW2 WS1", "HW2 WS2"};
    Visualizer::makeBoxPlot(rrt_all_path_lengths, labels, 
                               title, xlabel, ylabel); 
    xlabel = "Workspace";
    ylabel = "Number of Valid Solutions out of 100 runs";
    title = "RRT Number of Valid Solutions by workspace";
    // labels = {"n=200, r=0.5","n=200, r=1.0","n=200, r=1.5", "n=200, r=2.0", "n=500, r=0.5","n=500, r=1.0","n=500, r=1.5", "n=500, r=2.0"};
    // labels = {"n=200, r=1","n=200, r=2","n=500, r=1", "n=500, r=2", "n=1000, r=1","n=1000, r=2"};
    labels = {"HW5 WS1", "HW2 WS1", "HW2 WS2"};
    Visualizer::makeBarGraph(rrt_all_valid_sols, labels, 
                               title, xlabel, ylabel); 

    }


    Visualizer::showFigures();

    // Grade method
    // HW7::grade<MyPRM2D, MyRRT>("shaya.naimi@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}