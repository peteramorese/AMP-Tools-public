// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"

using namespace amp;

int main(int argc, char** argv) {
    // Load problem objects
    std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblem(), 
                                                  HW9::getFOUniProblem(), 
                                                  HW9::getSOUniProblem(), 
                                                  HW9::getCarInWS1(), 
                                                  HW9::getParkingProblem()};
    
    // Select problem, plan, check, and visualize
    KinodynamicProblem2D prob = problems[0];
    MyKinoRRT kino_planner;
    KinoPath path = kino_planner.plan(prob);
    HW9::check(path, prob);
    if (path.valid) 
        Visualizer::makeFigure(prob, path, prob.dimensions, prob.isPointAgent);
    Visualizer::showFigures();
    HW9::grade<MyKinoRRT>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple());
    return 0;
}