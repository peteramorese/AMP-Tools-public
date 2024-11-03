// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "hw/HW2.h"

#include "MyMultiAgentPlanners.h"

using namespace amp;

void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time elapsed: " << Profiler::getTotalProfile("timer") << std::endl;
}

int main(int argc, char** argv) {
    Problem2D prob = HW2::getWorkspace2();
    Visualizer::makeFigure(prob);
    Visualizer::showFigures();
    // Run timer example (useful for benchmarking)
    //timer_example();
    
    // Initialize Workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());
    MultiAgentProblem2D problem = HW8::getWorkspace1(2);

    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    // Solve using a centralized approach
    MyCentralPlanner central_planner;
    central_planner.myN = 7500;
    central_planner.myr = 0.5;
    // MultiAgentPath2D path = central_planner.plan(problem);
    // bool isValid = HW8::check(path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);
    // HW8::generateAndCheck(central_planner);
    // MultiAgentPath2D path = central_planner.plan(problem);
    std::cout << "Path waypoints: ";
    
  
    MyDecentralPlanner decentral_planner;
    
    // MultiAgentPath2D path = central_planner.plan(problem);
    // MultiAgentPath2D path;
    
    collision_states = {{}};


    std::list <std::vector<double>> all_computation_times;
    std::list <std::vector<double>> all_path_lengths;
    std::vector<double> all_valid_sols;
    bool bench = true;
    

    if (bench) {
        for (int m: {2, 3, 4, 5, 6}) {
            problem = HW8::getWorkspace1(m);
            central_planner.myN = 7500*m;
            std::cout << "m: " << m << std::endl;
            std::vector<double> computation_times;
            std::vector<double> tree_sizes;
            for (int i = 0; i < 100; ++i) {
                
                auto start = std::chrono::high_resolution_clock::now();
                        
                amp::MultiAgentPath2D testpath = decentral_planner.plan(problem);
                
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                computation_times.push_back(duration.count());
                int plength  = central_planner.treeSize;
                
                if (plength > 0) {
                    tree_sizes.push_back(plength);
                }
            }
            all_computation_times.push_back(computation_times);
            all_path_lengths.push_back(tree_sizes);
        }

        
    std::string xlabel = "Number of Agents";
    std::string ylabel = "Computation Time (microseconds)";
    std::string title = "Decentralized Planner Computation Time (HW8WS1)";
    std::vector<std::string> labels = {"m=2", "m=3", "m=4", "m=5", "m=6"};
    Visualizer::makeBoxPlot(all_computation_times, labels, 
                               title, xlabel, ylabel);  
std::vector<double> average_tree_sizes;
for (const auto& sizes : all_path_lengths) {
    double sum = std::accumulate(sizes.begin(), sizes.end(), 0.0);
    double average = sum / sizes.size();
    average_tree_sizes.push_back(average);
}
std::vector<double> average_comp_times;
for (const auto& times : all_computation_times) {
    double sum = std::accumulate(times.begin(), times.end(), 0.0);
    double average = sum / times.size();
    average_comp_times.push_back(average);
}
std::cout << "Average tree sizes: ";
for (const auto& avg : average_tree_sizes) {
std::cout << "Average computation times: ";
for (const auto& avg : average_comp_times) {
    std::cout << avg << " ";
}
std::cout << std::endl;
    xlabel = "Number of Agents";
    ylabel = "Tree Size";
    title = "Centralized Planner Tree Size (HW8WS1)";
    labels = {"m=2", "m=3", "m=4", "m=5", "m=6"};
    Visualizer::makeBoxPlot(all_path_lengths, labels, 
                               title, xlabel, ylabel); 
    
    
    }

    }


 

    
    // if (bench) {
    //     for (int N : {200, 500, 1000}){
    //         for (double r: {1, 2}){
    //         std::cout << "N: " << N << " r: " << r << std::endl;
            
    //         std::vector<double> path_lengths;
    //         int valid_sols = 0;
    //         std::vector<double> computation_times;
    //         prm.myN = N;
    //         prm.myr = r;
    //         for (int i = 0; i < 100; ++i) {
    //                 // std::cout<< "iteration: " << i << "\n";
    //                 auto start = std::chrono::high_resolution_clock::now();
                    
    //                 amp::Path2D path = prm.plan(problem);
                    
    //                 auto stop = std::chrono::high_resolution_clock::now();
    //                 auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //                 computation_times.push_back(duration.count());
    //                 double plength  = path.length();
                    
    //                 if (plength > 0) {
    //                     valid_sols++;
    //                     path_lengths.push_back(path.length());
    //                 }


    //             }
    //             all_computation_times.push_back(computation_times);
    //             all_path_lengths.push_back(path_lengths);
    //             all_valid_sols.push_back(valid_sols);
            
    //     }
    // }
    // //const std::list<std::vector<double>> values = all_path_lengths;    
    // std::string xlabel = "Hyperparameters n and r";
    // std::string ylabel = "Computation Time (microseconds)";
    // std::string title = "Smoothing PRM Computation Time (HW2WS2)";
    // std::vector<std::string> labels = {"n=200, r=1","n=200, r=2","n=500, r=1", "n=500, r=2", "n=1000, r=1","n=1000, r=2"};
    // Visualizer::makeBoxPlot(all_computation_times, labels, 
    //                            title, xlabel, ylabel);  
    // xlabel = "Hyperparameters n and r";
    // ylabel = "Path Length";
    // title = "Smoothing PRM Path Length (HW2WS2)";
    // // labels = {"n=200, r=0.5","n=200, r=1.0","n=200, r=1.5", "n=200, r=2.0", "n=500, r=0.5","n=500, r=1.0","n=500, r=1.5", "n=500, r=2.0"};
    // labels = {"n=200, r=1","n=200, r=2","n=500, r=1", "n=500, r=2", "n=1000, r=1","n=1000, r=2"};
    // Visualizer::makeBoxPlot(all_path_lengths, labels, 
    //                            title, xlabel, ylabel); 
    // xlabel = "Hyperparameters n and r";
    // ylabel = "Number of Valid Solutions out of 100 runs";
    // title = "Smoothing PRM Number of Valid Solutions (HW2WS2)";
    // // labels = {"n=200, r=0.5","n=200, r=1.0","n=200, r=1.5", "n=200, r=2.0", "n=500, r=0.5","n=500, r=1.0","n=500, r=1.5", "n=500, r=2.0"};
    // labels = {"n=200, r=1","n=200, r=2","n=500, r=1", "n=500, r=2", "n=1000, r=1","n=1000, r=2"};
    // Visualizer::makeBarGraph(all_valid_sols, labels, 
    //                            title, xlabel, ylabel); 

    // }


    
    // HW8::generateAndCheck(decentral_planner, path, problem, collision_states);
    // bool isValid = HW8::check(path, problem, collision_states);
    
    // Visualizer::makeFigure(problem, path, collision_states);

    // Visualize and grade methods
    Visualizer::showFigures();
    // HW8::grade<MyCentralPlanner, MyDecentralPlanner>("shaya.naimi@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}