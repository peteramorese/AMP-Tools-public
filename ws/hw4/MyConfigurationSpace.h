#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>

class MyConfigurationSpace: public amp::ConfigurationSpace2D{
    public:
       virtual bool inCollision(double x0, double x1) const{
            return true;
       }
    private:

};

class MyConfigEnvironment{
    public:
        amp::Polygon getCspaceObs(amp::Polygon robot, amp::Polygon obstacle){
                std::vector<Eigen::Vector2d> robotCVerts;
                std::vector<Eigen::Vector2d> obsCVerts;
                std::vector<Eigen::Vector2d> tempObs;
                //First point on robot is reference point
                for(int j = 0; j < robot.verticesCCW().size(); j++){
                    robotCVerts.push_back(robot.verticesCCW()[0] - robot.verticesCCW()[j]);

                    // robotCVerts.push_back()
                }
                //align new lower-left vertex as first 
                std::rotate(robotCVerts.begin(), robotCVerts.begin() + 1, robotCVerts.end());
                tempObs = obstacle.verticesCCW();
                // std::rotate(tempObs.rbegin(), tempObs.rbegin() + 1, tempObs.rend());
                int i = 0;
                int j = 0;
                int rSize = robotCVerts.size();
                int oSize = obstacle.verticesCW().size();
                double rAng;
                double oAng;
                do{
                    obsCVerts.push_back(robotCVerts[i%rSize] + tempObs[j%oSize]);
                    //TODO: angle wrapping?
                    rAng = getAngle(robotCVerts[(i + 1)%rSize],robotCVerts[i%rSize]);
                    oAng = getAngle(tempObs[(j + 1)%rSize],tempObs[j%rSize]);

                    i == rSize? rAng = getAngle(robotCVerts[(i + 1)%rSize],robotCVerts[i%rSize]) + 2*M_PI : 
                    rAng = getAngle(robotCVerts[(i + 1)%rSize],robotCVerts[i%rSize]);

                    j == oSize? oAng = getAngle(tempObs[(j + 1)%oSize],tempObs[j%oSize]) + 2*M_PI : 
                    oAng = getAngle(tempObs[(j + 1)%oSize],tempObs[j%oSize]);
                    std::cout<<"rAng: " << rAng << " oAng: " << oAng << std::endl;
                    if(rAng < oAng){
                        i += 1;
                    }else if(rAng > oAng){
                        j += 1;
                    }
                    else{
                        i += 1;
                        j += 1;
                    }
                    std::cout<< "i: " << i << " j: " << j << std::endl;
                }while(i < rSize || j < oSize);
                amp::Polygon obsC(obsCVerts);
                // amp::Polygon obsC(robotCVerts);
                for(int j = 0; j < robotCVerts.size(); j++){
                    std::cout << "robotCVerts[" << j << "] = " << robotCVerts[j] << std::endl;
                }
                for(int j = 0; j < tempObs.size(); j++){
                    std::cout << "tempObs[" << j << "] = " << tempObs[j] << std::endl;
                }
                return obsC;
        }
        double getAngle(Eigen::Vector2d fin, Eigen::Vector2d init){
            double ang = atan2(fin[1] - init[1],fin[0] - init[0]);
            if(ang < 0){
                return ang + 2*M_PI;
            }
            return ang;
        }
};