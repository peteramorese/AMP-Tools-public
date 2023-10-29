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
    MyPRM prm;
    prm.getN() = 200;
    prm.getR() = 1;
    prm.getS() = false;
    std::vector<Eigen::Vector2d> collision_points;
    amp::Problem2D w1 = HW5::getWorkspace1();
    w1.y_min = -3;
    w1.y_max = 3;
    amp::Path2D path = prm.plan(w1);
    HW7::check(path,w1,collision_points);
    Visualizer::makeFigure(w1, path, collision_points);
    prm.pathSmoother(w1,path);
    Visualizer::makeFigure(w1, path, collision_points);

    prm.getN() = 1000;
    prm.getR() = 2;
    collision_points.clear();
    amp::Problem2D w2 = HW2::getWorkspace1();
    path = prm.plan(w2);
    HW7::check(path,w2,collision_points);
    Visualizer::makeFigure(w2, path, collision_points);
    prm.pathSmoother(w2,path);
    Visualizer::makeFigure(w2, path, collision_points);
    
    collision_points.clear();
    amp::Problem2D w3 = HW2::getWorkspace2();
    path = prm.plan(w3);
    HW7::check(path,w3,collision_points);
    Visualizer::makeFigure(w3, path, collision_points);
    prm.pathSmoother(w3,path);
    Visualizer::makeFigure(w3, path, collision_points);


    Visualizer::showFigures();
    return 0;
}