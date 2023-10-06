#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>

class MyConfigurationSpace: public amp::ConfigurationSpace2D{
    public:
       virtual bool inCollision(double x0, double x1) const{
            
            // getJointLocation(const ManipulatorState& state, uint32_t joint_index)
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
                // for(int j = 0; j < robotCVerts.size(); j++){
                //     std::cout << "robotCVerts[" << j << "] = " << robotCVerts[j] << std::endl;
                // }
                tempObs = obstacle.verticesCCW();
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
                    // std::cout<<"rAng: " << rAng << " oAng: " << oAng << std::endl;
                    if(rAng < oAng){
                        i += 1;
                    }else if(rAng > oAng){
                        j += 1;
                    }
                    else{
                        i += 1;
                        j += 1;
                    }
                    // std::cout<< "i: " << i << " j: " << j << std::endl;
                }while(i < rSize || j < oSize);
                amp::Polygon obsC(obsCVerts);
                return obsC;
        }
        double getAngle(Eigen::Vector2d fin, Eigen::Vector2d init){
            double ang = atan2(fin[1] - init[1],fin[0] - init[0]);
            if (std::abs(ang) < 1e-10) ang = 0 ;
            if(ang < 0){
                return ang + 2*M_PI;
            }
            return ang;
        }
        void rotateRobot(std::vector<Eigen::Vector2d>& verts, double theta){
            Eigen::Matrix3d R3;
            R3 <<   cos(theta), -sin(theta), 0.0,
                    sin(theta), cos(theta),  0.0, 
                    0.0,    0.0,     1.0;
            Eigen::Vector3d  tempVec(0, 0, 1);
            for(int j = 0; j < verts.size(); j++){
                tempVec.head<2>() = verts[j];
                tempVec = R3*tempVec;
                if (std::abs(tempVec[0]) < 1e-10) tempVec[0] = 0 ;
                if (std::abs(tempVec[1]) < 1e-10) tempVec[1] = 0 ;
                verts[j] = tempVec.head<2>();
            }

        }
};