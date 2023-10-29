#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW7.h"
#include "MyPointMotionPlanner.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Visualizer::showFigures();
    // amp::HW6::grade(wf1g, wf2g, Astar, "collin.hudson@colorado.edu", argc, argv);
    // HW7::hint();
    amp::Problem2D w1 = HW5::getWorkspace1();
    amp::Problem2D w2 = HW2::getWorkspace1();
    amp::Problem2D w3 = HW2::getWorkspace2();
    amp::Path2D path;
    std::vector<Eigen::Vector2d> collision_points;
    {LOG("PRM");
        MyPRM prm;
        prm.getN() = 200;
        prm.getR() = 1;
        prm.getS() = false;
        w1.y_min = -3;
        w1.y_max = 3;
        path = prm.plan(w1);
        HW7::check(path,w1,collision_points);
        // Visualizer::makeFigure(w1, path, collision_points);
        prm.pathSmoother(w1,path);
        // Visualizer::makeFigure(w1, path, collision_points);
        prm.getN() = 150;
        prm.getR() = 5;
        prm.getW() = true;
        collision_points.clear();
        path = prm.plan(w2);
        HW7::check(path,w2,collision_points);
        // Visualizer::makeFigure(w2, path, collision_points);
        prm.pathSmoother(w2,path);
        // Visualizer::makeFigure(w2, path, collision_points);
        
        collision_points.clear();
        path = prm.plan(w3);
        HW7::check(path,w3,collision_points);
        // Visualizer::makeFigure(w3, path, collision_points);
        prm.pathSmoother(w3,path);
        // Visualizer::makeFigure(w3, path, collision_points);
    }
    {LOG("RRT");
        MyGoalBiasRRT RRT;
        collision_points.clear();
        path = RRT.plan(w1);
        HW7::check(path,w1,collision_points);
        Visualizer::makeFigure(w1, path, collision_points);
        collision_points.clear();
        path = RRT.plan(w2);
        HW7::check(path,w2,collision_points);
        Visualizer::makeFigure(w2, path, collision_points);
        collision_points.clear();
        path = RRT.plan(w3);
        HW7::check(path,w3,collision_points);
        Visualizer::makeFigure(w3, path, collision_points);
    }
    Visualizer::showFigures();
    // amp::HW7::grade(prm, RRT, "collin.hudson@colorado.edu", argc, argv);

    return 0;
}