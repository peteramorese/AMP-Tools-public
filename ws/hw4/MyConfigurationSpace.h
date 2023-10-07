#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>

class MyGridCSpace2D: public amp::GridCSpace2D{
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
        :GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max),dArr(x0_cells,x1_cells){
        }
        double getT(Eigen::Vector2d bugXY, Eigen::Vector2d bugNext, Eigen::Vector2d v1, Eigen::Vector2d v2) const {
            Eigen::Matrix2d Tmat1;
            Eigen::Matrix2d Tmat2;
            Tmat1.col(0) = bugXY-v1;
            Tmat1.col(1) = v1-v2;
            Tmat2.col(0) = bugXY-bugNext;
            Tmat2.col(1) = v1-v2;
            return Tmat1.determinant()/Tmat2.determinant();
        }
        double getU(Eigen::Vector2d bugXY, Eigen::Vector2d bugNext, Eigen::Vector2d v1, Eigen::Vector2d v2) const {
            Eigen::Matrix2d Tmat1;
            Eigen::Matrix2d Tmat2;
            Tmat1.col(0) = bugXY-v1;
            Tmat1.col(1) = bugXY-bugNext;
            Tmat2.col(0) = bugXY-bugNext;
            Tmat2.col(1) = v1-v2;
            return Tmat1.determinant()/Tmat2.determinant();
        }
        bool evalTU( double t, double u) const {
            return t >= 0 && t <= 1 && u >= 0 && u <= 1;
        }
        bool stepCheck(Eigen::Vector2d position, Eigen::Vector2d next, const amp::Environment2D& problem) const {
            bool hit = false;
            for(auto Ob : problem.obstacles){
                if(!hit){
                    for(int j = 0; j < Ob.verticesCCW().size(); j++){
                        if(!hit){
                            hit = checkHit(position,next,Ob,j); //hitPoint set in checkHit function
                        }
                    }
                }
            }
            return hit;
        }
        bool checkHit(Eigen::Vector2d position, Eigen::Vector2d next,amp::Polygon Ob, int startVert) const{
            int endVert;
            std::vector<Eigen::Vector2d> vertCW = Ob.verticesCW();
            int numV = vertCW.size() - 1;
            (startVert == 0) ? (endVert = numV) : endVert = startVert - 1;
            double t = getT(position,next,vertCW[startVert],vertCW[endVert]);
            double u = getU(position,next,vertCW[startVert],vertCW[endVert]);
            if((next - vertCW[startVert]).norm() < 0.005){
                return true;
            }
            else if(evalTU(t,u)){
                return true;
            }
            return false;
        }
        MyGridCSpace2D makeCSpace(MyLinkManipulator& mani, const amp::Environment2D& obs){
            //fills DenseArray2D by posing manipulator and checking for collision
            MyGridCSpace2D tempGrid(250,250,0,2*M_PI,0,2*M_PI);
            std::pair<std::size_t, std::size_t> siz = dArr.size();
            std::vector<double> state;
            for(int i = 0; i < siz.first; i++){
                for(int j = 0; j < siz.second; j++){
                    state.clear();
                    state.push_back(i*(x0Bounds().second - x0Bounds().first)/siz.first);
                    state.push_back(j*(x1Bounds().second - x1Bounds().first)/siz.second);
                    Eigen::Vector2d a0 = mani.getBaseLocation();
                    Eigen::Vector2d a1 =  mani.getJointLocation(state,1);
                    Eigen::Vector2d a2 = mani.getJointLocation(state,2);
                    bool hit = false;
                    for(auto Ob : obs.obstacles){
                        if(!hit){
                            for(int j = 0; j < Ob.verticesCCW().size(); j++){
                                if(!hit){
                                    hit = checkHit(a0,a1,Ob,j) || checkHit(a1,a2,Ob,j);
                                    // hit = checkHit(a1,a2,Ob,j);
                                }
                                else{
                                    // std::cout << "hit! for state: " << state[0] << " ," << state[1] << std::endl;
                                }
                            }
                        }
                    }
                    
                    dArr(i,j) =  hit;
                    tempGrid(i,j) = hit;
                }
            }
            return tempGrid;
            
        }
        virtual bool inCollision(double x0, double x1) const{
            std::pair<std::size_t, std::size_t> siz =  dArr.size();
            int i = ((x0 - x0Bounds().first)/(x0Bounds().second - x0Bounds().first))*siz.first; // (xspace0max - x0)/(xspace0max - xspace0min)* siz[0]
            int j = ((x1 - x1Bounds().first)/(x1Bounds().second - x1Bounds().first))*siz.second; // (xspace1max - x1)/(xspace1max - xspace1min)* siz[1]
            // std::cout << "x0: " << x0 << " x1: " << x1 << " i " << i << " j " << j << std::endl;
            return dArr(i,j);
        }
        amp::DenseArray2D<bool>& getdArr(){return dArr;};
        inline const amp::DenseArray2D<bool>& getdArr() const {return dArr;};
        
    private:
    amp::DenseArray2D<bool> dArr;
};

class MyGridCSpace2DConstructor: public amp::GridCSpace2DConstructor{
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override{
            std::unique_ptr<MyGridCSpace2D> ptr(new MyGridCSpace2D(250,250,0,2*M_PI,0,2*M_PI));
            MyLinkManipulator mani(manipulator.getBaseLocation(),manipulator.getLinkLengths());
            ptr->makeCSpace(mani, env);
            return ptr;
        }
};


class MyConfigEnvironment{
    public:
        amp::Polygon getCspaceObs(amp::Polygon robot, amp::Polygon obstacle){
                std::vector<Eigen::Vector2d> robotCVerts;
                std::vector<Eigen::Vector2d> obsCVerts;
                std::vector<Eigen::Vector2d> tempObs;
                //lowest left point on robot is reference point
                // int smallYIdx = 0;
                // for(int j = 1; j < robot.verticesCCW().size(); j++){
                //     if(robot.verticesCCW()[j][1] < robot.verticesCCW()[smallYIdx][1]){
                //         smallYIdx = j;
                //     }else if(robot.verticesCCW()[j][1] == robot.verticesCCW()[smallYIdx][1]){
                //         if(robot.verticesCCW()[j][0] < robot.verticesCCW()[smallYIdx][0]){
                //             smallYIdx = j;
                //         }
                //     }
                // }
                //Diff loop
                for(int j = 0; j < robot.verticesCCW().size(); j++){
                    robotCVerts.push_back(-robot.verticesCCW()[j]);
                }

                int smallYIdx = 0;
                for(int j = 1; j < robotCVerts.size(); j++){
                    if(robotCVerts[j][1] < robotCVerts[smallYIdx][1]){
                        smallYIdx = j;
                    }else if(robotCVerts[j][1] == robotCVerts[smallYIdx][1]){
                        if(robotCVerts[j][0] < robotCVerts[smallYIdx][0]){
                            smallYIdx = j;
                        }
                    }
                }
                //align new lower-left vertex as first 
                std::rotate(robotCVerts.begin(), robotCVerts.begin() + smallYIdx, robotCVerts.end());
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
                    rAng = getAngle(robotCVerts[(i + 1)%rSize],robotCVerts[i%rSize]) + i == rSize ? 2*M_PI : 0;
                    oAng = getAngle(tempObs[(j + 1)%rSize],tempObs[j%rSize]) + j == rSize ? 2*M_PI : 0;
        
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