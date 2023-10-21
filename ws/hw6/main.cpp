#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW4.h"
#include "hw/HW6.h"
#include "MyLinkManipulator.h"
#include "MyConfigurationSpace.h"
// #include "MyGridCSpace.h"
#include "MyAlgos.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    LOG("Problem 1");
    {
        const Problem2D problem1 = HW2::getWorkspace1();
        MyPointWFAlgo wf1;
        const std::unique_ptr<amp::GridCSpace2D> grid_cspace1 = wf1.constructDiscretizedWorkspace(problem1);
        auto path1 = wf1.planInCSpace(problem1.q_init,problem1.q_goal,*grid_cspace1);    
        Visualizer::makeFigure(problem1,path1);
        // MyGridCSpace2D plot1(std::ceil((problem1.x_max - problem1.x_min)/0.25),std::ceil((problem1.y_max - problem1.y_min)/0.25),problem1.x_min,problem1.x_max,problem1.y_min,problem1.y_max);
        // Visualizer::makeFigure(plot1.makeCSpacePoint(problem1),path1);

        HW6::checkPointAgentPlan(path1,problem1);
        LOG("path length: " << path1.length());
        {
            bool random_trial_success = true;
            // while(random_trial_success){
                amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
                amp::Problem2D random_prob; 
                std::vector<Eigen::Vector2d> collision_points;
                
                random_trial_success = HW6::generateAndCheck(wf1, path, random_prob, collision_points);
                // LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));
                LOG("path length: " << path.length());

                // Visualize the path environment, and any collision points with obstacles
                // if(!random_trial_success){
                    Visualizer::makeFigure(random_prob, path, collision_points);
                // }
            // }
        }
    }

    LOG("Problem 2");
    {
        MyGridCSpace2DConstructor cons;
        cons.getGridWidth() = 0.25;
        MyManipWFAlgo wf2(cons);
        MyLinkManipulator mani;
        const amp::Problem2D problem2 = HW6::getHW4Problem3();
        auto path = wf2.plan(mani,problem2);
        MyGridCSpace2D plotEnv3(std::ceil((cons.getX0_bounds().second - cons.getX0_bounds().first)/cons.getGridWidth()),std::ceil((cons.getX1_bounds().second - cons.getX1_bounds().first)/cons.getGridWidth()),cons.getX0_bounds().first,cons.getX0_bounds().second,cons.getX1_bounds().first,cons.getX1_bounds().second);
        Visualizer::makeFigure(plotEnv3.makeCSpace(mani,problem2),path);

        Eigen::Vector2d lowerB(cons.getX0_bounds().first,cons.getX1_bounds().first);
        Eigen::Vector2d upperB(cons.getX0_bounds().second,cons.getX1_bounds().second);
        unwrapPath(path,lowerB,upperB);
        Visualizer::makeFigure(problem2,mani,path);
        HW6::checkLinkManipulatorPlan(path,mani,problem2);
        LOG("path length: " << path.length());
        {
            // bool random_trial_success = true;
            // while(random_trial_success){
            //     amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
            //     amp::Problem2D random_prob; 
            //     std::vector<Eigen::Vector2d> collision_points;
            //     MyLinkManipulator mani;
            //     MyGridCSpace2DConstructor consR;
            //     cons.getGridWidth() = 0.05;
            //     MyManipWFAlgo wfr(consR);
            //     random_trial_success = HW6::generateAndCheck(wfr,mani,path,random_prob,collision_points);
            //     LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));
            //     LOG("path length: " << path.length());

            //     // Visualize the path environment, and any collision points with obstacles
            //     // if(random_trial_success){
            //         MyGridCSpace2D plotEnv3(std::ceil((consR.getX0_bounds().second - consR.getX0_bounds().first)/consR.getGridWidth()),std::ceil((consR.getX1_bounds().second - consR.getX1_bounds().first)/consR.getGridWidth()),consR.getX0_bounds().first,consR.getX0_bounds().second,consR.getX1_bounds().first,consR.getX1_bounds().second);
            //         Visualizer::makeFigure(plotEnv3.makeCSpace(mani,random_prob),path);
            //     // }
            // }
        }
    }

    LOG("Problem 3");
    




    Visualizer::showFigures();
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("collin.hudson@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}