// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "KinoRRT.h"
#include "Triangulate.h"
#include "SynergisticPlanner.h"
#include "KinoRRT.h"
#include "MyAStar.h"
#include "DFA.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

using namespace amp;

void writeWaypointsToCSV(const std::vector<Eigen::VectorXd>& waypoints, const std::string& filename) {
    std::ofstream file(filename); // Open file for writing

    if (file.is_open()) {
        for (const auto& waypoint : waypoints) {
            for (int i = 0; i < waypoint.size(); ++i) {
                file << waypoint(i);
                if (i < waypoint.size() - 1) {
                    file << ","; // Add comma separator between elements
                }
            }
            file << "\n"; // Start a new line for the next waypoint
        }
        file.close(); // Close the file
        std::cout << "Waypoints written to " << filename << std::endl;
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}


void problem1() {
    Problem2D problem = HW2::getWorkspace1();
    Eigen::VectorXd initState(5);
    initState << problem.q_init(0), problem.q_init(1), 0.0, 0.0, 0.0;
    pair<double, double> velocityLimit = {-1/3.0, 1/2.0};
    pair<double, double> steeringLimit = {-M_PI/3.0, M_PI/3.0};
    vector<pair<double, double>> stateLimits = {{problem.x_min, problem.x_max}, {problem.y_min, problem.y_max}, 
                                                {0, 2.0*M_PI}, {-1/6.0, 1/2.0}, {-M_PI/6.0, M_PI/6.0}};
    vector<pair<double, double>> controlLimits = {{-1.0/2.0, 1.0/2.0}, {-M_PI/3.0, M_PI/3.0}};

    std::vector<Eigen::Vector2d> workspaceVertices = {
        Eigen::Vector2d(problem.x_min, problem.y_min),
        Eigen::Vector2d(problem.x_max, problem.y_min),
        Eigen::Vector2d(problem.x_max, problem.y_max),
        Eigen::Vector2d(problem.x_min, problem.y_max)
    };

    std::vector<Eigen::Vector2d> taskA = {
        Eigen::Vector2d(10, 0),
        Eigen::Vector2d(12, 0),
        Eigen::Vector2d(12, 2),
        Eigen::Vector2d(10, 2)
    };

    std::vector<Eigen::Vector2d> taskG = {
        Eigen::Vector2d(9, 9),
        Eigen::Vector2d(11, 9),
        Eigen::Vector2d(11, 11),
        Eigen::Vector2d(9, 11)
    };

    std::vector<std::pair<std::vector<Eigen::Vector2d>, char>> polygons = {{taskA, 'a'}, {taskG, 'g'}};
    for (auto poly : problem.obstacles) {
        polygons.push_back({poly.verticesCCW(), 'o'});
        std::cout << "Obstacle\n";
        for (const auto& vec : poly.verticesCCW()) 
            std::cout << "(" << vec.x() << ", " << vec.y() << ")" << std::endl;
    }

    const std::unordered_map<uint32_t, abstractionNode> M = triangulatePolygon(workspaceVertices, polygons, 3);
    const DFA A = createDFA();
    SynergisticPlanner planner(A, M, 5000, 0.5, 0.15, controlLimits);
    MyKinoChecker kinoChecker(problem, stateLimits, 0.5, 1);
    amp::Path path = planner.synergisticPlan(initState, kinoChecker);

    if (path.waypoints.size() != 0) {
        Path2D path2d;
        for (const VectorXd point : path.waypoints) path2d.waypoints.push_back({point(0), point(1)});
        Visualizer::makeFigure(problem, path2d);
        writeWaypointsToCSV(path.waypoints, "waypoints.csv");
    }
}

void problem2() {
    Eigen::VectorXd initState(5);
    initState << 18.0, 18.0, -1.571, 0.0, 0.0;
    pair<double, double> steeringLimit = {-M_PI/3.0, M_PI/3.0};
    vector<pair<double, double>> stateLimits = {{0.0, 20.0}, {0.0, 20.0}, 
                                                {0, 2.0*M_PI}, {-1/6.0, 1/2.0}, {-M_PI/4.0, M_PI/4.0}};
    vector<pair<double, double>> controlLimits = {{-1.0/2.0, 1.0/2.0}, {-M_PI/4.0, M_PI/4.0}};

    std::vector<Eigen::Vector2d> workspaceVertices = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(20.0, 0.0),
        Eigen::Vector2d(20.0, 20.0),
        Eigen::Vector2d(0.0, 20.0)
    };

    std::vector<Eigen::Vector2d> taskA = {
        Eigen::Vector2d(15, 1),
        Eigen::Vector2d(17, 1),
        Eigen::Vector2d(17, 3),
        Eigen::Vector2d(15, 3)
    };

    std::vector<Eigen::Vector2d> taskB = {
        Eigen::Vector2d(5, 1),
        Eigen::Vector2d(7, 1),
        Eigen::Vector2d(7, 3),
        Eigen::Vector2d(5, 3)
    };

    std::vector<Eigen::Vector2d> taskG = {
        Eigen::Vector2d(4, 14),
        Eigen::Vector2d(6, 14),
        Eigen::Vector2d(6, 16),
        Eigen::Vector2d(4, 16)
    };

    std::vector<std::vector<Eigen::Vector2d>> walls = {
       {
        Eigen::Vector2d(0, 10),
        Eigen::Vector2d(7, 10), 
        Eigen::Vector2d(7, 12),
        Eigen::Vector2d(0, 12)
      },
      {
        Eigen::Vector2d(10, 12),
        Eigen::Vector2d(12, 12),
        Eigen::Vector2d(12, 20),
        Eigen::Vector2d(10, 20)
      },
      {
        Eigen::Vector2d(14, 8),
        Eigen::Vector2d(20, 8),
        Eigen::Vector2d(20, 10),
        Eigen::Vector2d(14, 10)
      },
      {
        Eigen::Vector2d(12, 0),
        Eigen::Vector2d(14, 0),
        Eigen::Vector2d(14, 4),
        Eigen::Vector2d(12, 4)
      } 
    };

    std::vector<std::pair<std::vector<Eigen::Vector2d>, char>> polygons = {{taskA, 'a'}, {taskB, 'b'}, {taskG, 'g'}};
    for (auto wall : walls) 
        polygons.push_back({wall, 'o'});

    const std::unordered_map<uint32_t, abstractionNode> M = triangulatePolygon(workspaceVertices, polygons, 2);
    const DFA A = createDFA();
    SynergisticPlanner planner(A, M, 10000, 0.5, 0.15, controlLimits);
    MyKinoChecker kinoChecker(walls, stateLimits, 0.5, 1);
    amp::Path path = planner.synergisticPlan(initState, kinoChecker);

    if (path.waypoints.size() != 0) {
        Path2D path2d;
        for (const VectorXd point : path.waypoints) path2d.waypoints.push_back({point(0), point(1)});
        // Visualizer::makeFigure(problem, path2d);
        writeWaypointsToCSV(path.waypoints, "waypoints.csv");
    }
}


void problem3() {
    Eigen::VectorXd initState(6);
    initState << 1.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    vector<pair<double, double>> stateLimits = {{0.0, 10.0}, {-1/2.0, 1/2.0}, {0.0, 10.0}, {-1/2.0, 1/2.0}, {0.0, 10.0}, {-1/2.0, 1/2.0}};
    vector<pair<double, double>> controlLimits = {{-1.0/4.0, 1.0/4.0}, {-1.0/4.0, 1.0/4.0}, {-1.0/4.0, 1.0/4.0}, {-1.0/4.0, 1.0/4.0}};

    std::vector<Eigen::Vector3d> workspaceVertices = {
        Eigen::Vector3d(0.0, 0.0, 0.0),  
        Eigen::Vector3d(10.0, 0.0, 0.0), 
        Eigen::Vector3d(10.0, 10.0, 0.0),
        Eigen::Vector3d(0.0, 10.0, 0.0), 
        Eigen::Vector3d(0.0, 0.0, 10.0), 
        Eigen::Vector3d(10.0, 0.0, 10.0),
        Eigen::Vector3d(10.0, 10.0, 10.0),
        Eigen::Vector3d(0.0, 10.0, 10.0) 
    };

    std::vector<Eigen::Vector3d> taskA = {
        Eigen::Vector3d(7.0, 2.0, 2.0),  // Vertex 1 (corner at (7, 2, 2))
        Eigen::Vector3d(8.0, 2.0, 2.0),  // Vertex 2
        Eigen::Vector3d(8.0, 3.0, 2.0),  // Vertex 3
        Eigen::Vector3d(7.0, 3.0, 2.0),  // Vertex 4
        Eigen::Vector3d(7.0, 2.0, 3.0),  // Vertex 5
        Eigen::Vector3d(8.0, 2.0, 3.0),  // Vertex 6
        Eigen::Vector3d(8.0, 3.0, 3.0),  // Vertex 7
        Eigen::Vector3d(7.0, 3.0, 3.0)   // Vertex 8
    };

    // std::vector<Eigen::Vector2d> taskG = {
    //     Eigen::Vector2d(4, 14),
    //     Eigen::Vector2d(6, 14),
    //     Eigen::Vector2d(6, 16),
    //     Eigen::Vector2d(4, 16)
    // };

    std::vector<std::pair<std::vector<Eigen::Vector3d>, char>> polytopes = {{taskA, 'a'}};
    // for (auto wall : walls) polytopes.push_back({wall, 'o'});


    const std::unordered_map<uint32_t, Node3D> M = triangulate3D(workspaceVertices, polytopes, 2);
    const DFA A = createDFA();
    SynergisticPlanner3D planner(A, M, 10000, 0.5, 0.15, controlLimits);
    MyKinoChecker kinoChecker({taskA}, stateLimits, 0.5, 0.5, 0.2);
    // amp::Path path = planner.synergisticPlan(initState, kinoChecker);

    // if (path.waypoints.size() != 0) {
    //     Path2D path2d;
    //     for (const VectorXd point : path.waypoints) path2d.waypoints.push_back({point(0), point(1)});
    //     // Visualizer::makeFigure(problem, path2d);
    //     writeWaypointsToCSV(path.waypoints, "waypoints.csv");
    // }
}

int main(int argc, char** argv) {
    problem2();
    // Visualizer::showFigures();
    return 0;
}