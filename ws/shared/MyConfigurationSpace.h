#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>
#include <math.h>

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
            std::pair<std::size_t, std::size_t> siz = dArr.size();
            MyGridCSpace2D tempGrid(siz.first,siz.second,x0Bounds().first, x0Bounds().second, x1Bounds().first, x1Bounds().second);
            std::vector<double> state;
            for(int i = 0; i < siz.first; i++){
                for(int j = 0; j < siz.second; j++){
                    state.clear();
                    state.push_back(i*(x0Bounds().second - x0Bounds().first)/siz.first);
                    state.push_back(j*(x1Bounds().second - x1Bounds().first)/siz.second);
                    double* ptr = &state[0];
                    Eigen::Map<Eigen::VectorXd> stateE(ptr, state.size());
                    Eigen::Vector2d a0 = mani.getBaseLocation();
                    Eigen::Vector2d a1 =  mani.getJointLocation(stateE,1);
                    Eigen::Vector2d a2 = mani.getJointLocation(stateE,2);
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
        MyGridCSpace2D makeCSpacePoint(const amp::Environment2D& obs){
            //fills DenseArray2D by posing manipulator and checking for collision
            std::pair<std::size_t, std::size_t> siz = dArr.size();
            MyGridCSpace2D tempGrid(siz.first,siz.second,x0Bounds().first, x0Bounds().second, x1Bounds().first, x1Bounds().second);
            std::vector<double> state;
            for(int i = 0; i < siz.first; i++){
                for(int j = 0; j < siz.second; j++){
                    state.clear();
                    double x = ((i+0.5)*(x0Bounds().second - x0Bounds().first)/siz.first + x0Bounds().first);
                    double y = ((j+0.5)*(x1Bounds().second - x1Bounds().first)/siz.second + x1Bounds().first);
                    bool hit = false;
                    Eigen::Vector2d nextVertex;
                    Eigen::Vector2d currentVertex;
                    // Point-in Polygon test modified from https://stackoverflow.com/a/34689268, retrieved 10/20/2023
                    for(auto Ob : obs.obstacles){
                        if(!hit){
                            hit = false;
                            int pos = 0;
                            int neg = 0;
                            for(int j = 0; j < Ob.verticesCCW().size(); j++){
                                nextVertex = Ob.verticesCCW()[(j + 1)%Ob.verticesCCW().size()];
                                currentVertex = Ob.verticesCCW()[j];
                                double d = (x - currentVertex(0))*(nextVertex(1) - currentVertex(1)) - (y - currentVertex(1))*(nextVertex(0) - currentVertex(0));

                                if (d > 0) pos++;
                                if (d < 0) neg++;

                                //If the sign changes, then point is outside
                                if (pos > 0 && neg > 0){
                                    hit = false;
                                }
                                else{
                                    hit = true;
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
        virtual bool inCollision(double x0, double x1) const override{
            std::pair<std::size_t, std::size_t> siz =  dArr.size();
            std::pair<std::size_t, std::size_t> cell = getCellFromPoint(x0,x1);
            return dArr(cell.first,cell.second);
        }
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override{
            std::pair<std::size_t, std::size_t> siz =  dArr.size();
            int i = ((x0 - x0Bounds().first)/(x0Bounds().second - x0Bounds().first))*siz.first; // (xspace0max - x0)/(xspace0max - xspace0min)* siz[0]
            int j = ((x1 - x1Bounds().first)/(x1Bounds().second - x1Bounds().first))*siz.second; // (xspace1max - x1)/(xspace1max - xspace1min)* siz[1]
            std::pair<std::size_t, std::size_t> cell(i,j);
            return cell;
        }
        amp::DenseArray2D<bool>& getdArr(){return dArr;};
        inline const amp::DenseArray2D<bool>& getdArr() const {return dArr;};
        
    private:
    amp::DenseArray2D<bool> dArr;
};

class MyGridCSpace2DConstructor: public amp::GridCSpace2DConstructor{
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override{
            LOG("constructing manipulator C-space...");

            std::unique_ptr<MyGridCSpace2D> ptr(new MyGridCSpace2D(std::ceil((x0_bounds.second - x0_bounds.first)/gridWidth),std::ceil((x1_bounds.second - x1_bounds.first)/gridWidth),x0_bounds.first,x0_bounds.second,x1_bounds.first,x1_bounds.second));
            //construct custom class version of manipulator
            MyLinkManipulator mani(manipulator.getBaseLocation(),manipulator.getLinkLengths());
            ptr->makeCSpace(mani, env);

            LOG("done constructing!");
            return ptr;
        }

        std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env){
            LOG("constructing point C-space...");
            std::unique_ptr<MyGridCSpace2D> ptr(new MyGridCSpace2D(std::ceil((x0_bounds.second - x0_bounds.first)/gridWidth),std::ceil((x1_bounds.second - x1_bounds.first)/gridWidth),x0_bounds.first,x0_bounds.second,x1_bounds.first,x1_bounds.second));
            ptr->makeCSpacePoint(env);
            
            LOG("done constructing!");
            return ptr;
        }
        
        double& getGridWidth(){return gridWidth;};
        inline const double& getGridWidth() const {return gridWidth;};

        std::pair<double, double>& getX0_bounds(){return x0_bounds;};
        inline const std::pair<double, double>& getX0_bounds() const {return x0_bounds;};

        std::pair<double, double>& getX1_bounds(){return x1_bounds;};
        inline const std::pair<double, double>& getX1_bounds() const {return x1_bounds;};
    private:
        double gridWidth = 0.25;
        std::pair<double, double> x0_bounds {0.0,2*M_PI};
        std::pair<double, double> x1_bounds {0.0,2*M_PI};
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