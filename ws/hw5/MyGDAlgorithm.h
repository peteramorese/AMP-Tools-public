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
            Eigen::Vector2d c(100,100);
            Eigen::Vector2d tempC;
            //TODO: determine closest point from XY to obstacle
            double CWAng;
            double CCWAng;
            double XYAng;
            int CWIdx;
            int CCWIdx;
            double t;
            bool corner;
            std::vector<Eigen::Vector2d> vCCW = Ob.verticesCCW();
            for(int j = 0; j < vCCW.size(); j++){

                j == 0 ? CWIdx = vCCW.size() - 1 : CWIdx = j - 1;
                j == vCCW.size() - 1 ? CCWIdx = 0 : CCWIdx = j + 1;
                // std::cout << "vCCW[j] " << vCCW[j] << " vCCW[CWIdx] " << vCCW[CWIdx] << " vCCW[CCWIdx] " << vCCW[CCWIdx] << " xy " << qXY << std::endl;
                //get angle between corner and cw vertex
                CWAng = atan2(vCCW[CWIdx](1) - vCCW[j](1), vCCW[CWIdx](0) - vCCW[j](0)) + M_PI/2;
                if(CWAng < 0){CWAng = CWAng + 2*M_PI;}
                //get angle between corner and ccw vertex
                CCWAng = atan2(vCCW[CCWIdx](1) - vCCW[j](1), vCCW[CCWIdx](0) - vCCW[j](0)) - M_PI/2;
                if(CCWAng <= 0){CCWAng = CCWAng + 2*M_PI;}
                //get angle between corner and current xy
                XYAng = atan2(qXY(1) -  vCCW[j](1), qXY(0) -  vCCW[j](0));
                if(XYAng < 0){XYAng = XYAng + 2*M_PI;}
                // std::cout << "vCCW[j] " << vCCW[j] << " CWAng " << CWAng << " CCWAng " << CCWAng << " XYAng " << XYAng << " xy " << qXY << std::endl;
                // std::cout << " XYAng >= CWAng " << (XYAng >= CWAng) << " XYAng <= CCWAng " << (XYAng <= CCWAng) << " (XYAng >= CWAng && XYAng <= CCWAng) " << (XYAng >= CWAng && XYAng <= CCWAng) << std::endl;
                // CWAng == 0 ? CWAng = 2*M_PI : CWAng = CWAng;
                // if((XYAng >= CWAng && XYAng <= CCWAng) || std::abs(XYAng - CCWAng) < 0.001 || std::abs(XYAng - CWAng) < 0.001){
                if((XYAng >= CWAng && XYAng <= CCWAng)){
                    // std::cout << "here1 vCCW[j] " << vCCW[j] << " CWAng " << CWAng << " CCWAng " << CCWAng << " XYAng " << XYAng << " xy " << qXY << std::endl;
                    // std::cout << "here1" << std::endl;
                    tempC = vCCW[j];
                    if((tempC - qXY).norm() < (c - qXY).norm() || c == qXY){
                        // std::cout<< "here1 " << "vCCW[j] " << vCCW[j] << " vCCW[CWIdx] " << vCCW[CWIdx] << " vCCW[CCWIdx] " << vCCW[CCWIdx] << " c " << tempC << std::endl;
                        c = tempC;
                    }
                    // else{
                    //     std::cout << " SAD1: vCCW[j] " << vCCW[j] << " vCCW[CWIdx] " << vCCW[CWIdx] << " vCCW[CCWIdx] " << vCCW[CCWIdx] << std::endl;
                    //     std::cout << "CWAng " << CWAng << " CCWAng " << CCWAng << " XYAng " << XYAng << std::endl;
                    // }
                }
                else if(XYAng >= CCWAng){
                    //Get normal from corner to CCW
                    CCWAng = atan2(vCCW[CCWIdx](1) - vCCW[j](1), vCCW[CCWIdx](0) - vCCW[j](0));
                    t = cos(2*M_PI - XYAng + CCWAng)*(vCCW[j] - qXY).norm()/(vCCW[j] - vCCW[CCWIdx]).norm();
                    // std::cout << "cos(2*M_PI - XYAng + CCWAng) " << cos(2*M_PI - XYAng + CCWAng) << " (vCCW[j] - qXY).norm() " << (vCCW[j] - qXY).norm() << " (vCCW[j] - vCCW[CCWIdx]).norm() " << (vCCW[j] - vCCW[CCWIdx]).norm() << " t " << t << std::endl;
                    if(t >= 0 && t <= 1 ){
                        // std::cout << "Here2 " << " t " << t << " xy " << qXY << std::endl;
                        tempC = ((1 - t)*vCCW[j] + t*vCCW[CCWIdx]);
                        if((tempC - qXY).norm() < (c - qXY).norm() || c == qXY){
                            // std::cout<< "here2, t " << t << " vCCW[j] " << vCCW[j] << " vCCW[CWIdx] " << vCCW[CWIdx] << " vCCW[CCWIdx] " << vCCW[CCWIdx] << " c " << tempC << std::endl;
                            c = tempC;
                        }
                        // else{
                        //     std::cout << " SAD2: vCCW[j] " << vCCW[j] << " vCCW[CWIdx] " << vCCW[CWIdx] << " vCCW[CCWIdx] " << vCCW[CCWIdx] << std::endl;
                        //     std::cout << "CWAng " << CWAng << " CCWAng " << CCWAng << " XYAng " << XYAng << std::endl;
                        // }
                    }
                }
                else if(XYAng <= CWAng){
                    //Get normal from corner to CW
                    CWAng = atan2(vCCW[CWIdx](1) - vCCW[j](1), vCCW[CWIdx](0) - vCCW[j](0));
                    t = cos(XYAng - CWAng)*(vCCW[j] - qXY).norm()/(vCCW[j] - vCCW[CWIdx]).norm();
                    if(t >= 0 && t <= 1 ){
                        // std::cout << "Here3 " << " t " << t << " xy " << qXY << std::endl;
                        tempC = ((1 - t)*vCCW[j] + t*vCCW[CWIdx]);
                        if((tempC - qXY).norm() < (c - qXY).norm() || c == qXY){
                            // std::cout<< "here3, t " << t << " vCCW[j] " << vCCW[j] << " vCCW[CWIdx] " << vCCW[CWIdx] << " vCCW[CCWIdx] " << vCCW[CCWIdx] << " c " << tempC << std::endl;
                            c = tempC;
                        }
                        // else{
                        //     std::cout << " SAD3: vCCW[j] " << vCCW[j] << " vCCW[CWIdx] " << vCCW[CWIdx] << " vCCW[CCWIdx] " << vCCW[CCWIdx] << " xy " << qXY << std::endl;
                        //     std::cout << "CWAng " << CWAng << " CCWAng " << CCWAng << " XYAng " << XYAng << std::endl;
                        // }
                    }
                }
                else{
                    std::cout << " SAD: vCCW[j] " << vCCW[j] << " vCCW[CWIdx] " << vCCW[CWIdx] << " vCCW[CCWIdx] " << vCCW[CCWIdx] << " xy " << qXY << std::endl;
                    std::cout << "CWAng " << CWAng << " CCWAng " << CCWAng << " XYAng " << XYAng << std::endl;
                }
            }
            if(c == qXY*10000){
                std::cout << "FAIL :((" << std::endl;
                std::cout << "vCCW[0] " << vCCW[0] << " xy " << qXY << std::endl;
            }
            // std::cout << "vCCW[0] " << vCCW[0] << " xy " << qXY << " c " << c << std::endl;
            return c - qXY;
        }
        Eigen::Vector2d getGradient(Eigen::Vector2d qXY, const amp::Problem2D& problem){
            //Get the current gradient vector
            Eigen::Vector2d UAttG(0,0);
            Eigen::Vector2d URepG(0,0);
            //Get gradient UAtt
            Eigen::Vector2d deltaGoal = (qXY - problem.q_goal);
            // std::cout << "deltaGoal " << deltaGoal << std::endl;
            if(deltaGoal.norm() <= dStarGoal){
                UAttG = xi*deltaGoal;
            }
            else{
                UAttG = (dStarGoal*xi/deltaGoal.norm())*deltaGoal;
            }
            // std::cout << "UAttG " << UAttG << std::endl;
            //Get URep
            Eigen::Vector2d obVec; //vector from qXY to closest point on obstacle
            for(int j = 0; j < problem.obstacles.size(); j++){
                obVec = vecToObs(qXY,problem.obstacles[j]);
                // std::cout << "obVec " << obVec << " xy " << qXY << std::endl;
                if(obVec.norm() <= QStar){
                    URepG += (eta*(1/QStar - 1/obVec.norm())/pow(obVec.norm(),2))*-obVec/obVec.norm();
                    // std::cout << "URepG " << URepG << " xy " << qXY << std::endl;
                }
            }
            // std::cout << "URepG Final " << URepG << " grad " << UAttG + URepG <<  " xy " << qXY << std::endl;
            return UAttG + URepG;
        }
        
    private:
        const double dStarGoal = 2;
        const double QStar = 0.25;
        const double xi = 1.5;
        const double eta = 1;
        
};