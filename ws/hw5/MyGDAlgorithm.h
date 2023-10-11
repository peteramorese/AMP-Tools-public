#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"
#include <Eigen/LU>

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyGDAlgorithm : public amp::GDAlgorithm {
    public:
        Eigen::Vector2d currentXY;
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Split up workspace into regions in front of vertices and in front of edges.
        /*
              v     edge
                 __________ vertex region     
               /           |
         edge /            | edge region
             /             |
            /______________|
           v      edge      v 
        */
        // Allows you to check distance by either using just the vertex or have to use normal vecs to edge

        // Don't need to create vector field plots

        //benchmarks only really worth extra credit 

        Eigen::Vector2d vecToObs(Eigen::Vector2d qXY, amp::Polygon Ob){
            //Get vector from qXY to closest point on an obstacle
            Eigen::Vector2d obVec(20,20);
            //TODO: determine closest point from XY to obstacle
            return obVec;
        }
        Eigen::Vector2d getGradient(Eigen::Vector2d qXY, const amp::Problem2D& problem){
            //Get the current gradient vector
            Eigen::Vector2d UAttG(0,0);
            Eigen::Vector2d URepG(0,0);
            //Get gradient UAtt
            Eigen::Vector2d deltaGoal = (qXY - problem.q_goal);
            std::cout << "deltaGoal " << deltaGoal << std::endl;
            if(deltaGoal.norm() <= dStarGoal){
                UAttG = xi*deltaGoal;
            }
            else{
                UAttG = (dStarGoal*xi/deltaGoal.norm())*deltaGoal;
            }
            std::cout << "UAttG " << UAttG << std::endl;
            //Get URep
            Eigen::Vector2d obVec; //vector from qXY to closest point on obstacle
            for(auto ob : problem.obstacles){
                obVec = vecToObs(qXY,ob);
                if(obVec.norm() <= QStar){
                    URepG += (eta*(1/QStar - 1/obVec.norm())/pow(obVec.norm(),2))*-obVec/obVec.norm();
                }
            }
            std::cout << "URepG " << URepG << std::endl;
            return UAttG + URepG;
        }
        
    private:
        const double dStarGoal = 2;
        const double QStar = 0.5;
        const double xi = 0.25;
        const double eta = 0.25;
        
};