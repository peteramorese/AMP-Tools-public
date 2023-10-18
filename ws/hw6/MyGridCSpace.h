#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include "hw/HW6.h"
#include <Eigen/LU>

class MyGridCSpace: public amp::GridCSpace2D{
    public:
        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
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
        MyGridCSpace makeCSpace(MyLinkManipulator& mani, const amp::Environment2D& obs){
            //fills DenseArray2D by posing manipulator and checking for collision
            MyGridCSpace tempGrid(250,250,0,2*M_PI,0,2*M_PI);
            std::pair<std::size_t, std::size_t> siz = dArr.size();
            std::vector<double> state;
            for(int i = 0; i < siz.first; i++){
                for(int j = 0; j < siz.second; j++){
                    state.clear();
                    state.push_back(i*(x0Bounds().second - x0Bounds().first)/siz.first);
                    state.push_back(j*(x1Bounds().second - x1Bounds().first)/siz.second);
                    double* ptr = &state[0];
                    Eigen::Map<Eigen::VectorXd> stateE(ptr, 4);
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
        virtual bool inCollision(double x0, double x1) const override{
            std::pair<std::size_t, std::size_t> siz =  dArr.size();
            std::pair<std::size_t, std::size_t> cell = getCellFromPoint(x0,x1);
            return dArr(cell.first,cell.second);
        }
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override{
            std::pair<std::size_t, std::size_t> siz =  dArr.size();
            int i = ((x0 - x0Bounds().first)/(x0Bounds().second - x0Bounds().first))*siz.first; // (x0 - xspace0min)/(xspace0max - xspace0min)* siz[0]
            int j = ((x1 - x1Bounds().first)/(x1Bounds().second - x1Bounds().first))*siz.second; // (x1 - xspace1min)/(xspace1max - xspace1min)* siz[1]
            std::pair<std::size_t, std::size_t> cell(i,j);
            return cell;
        }
        Eigen::Vector2d getPointFromCell(std::pair<std::size_t, std::size_t> cell) const{
            //Return midpoint of cell
            Eigen::Vector2d pt(0,0);
            std::pair<std::size_t, std::size_t> siz =  dArr.size();
            pt(0) = ((x0Bounds().second - x0Bounds().first)/siz.first)*(double(cell.first) - 0.5) + x0Bounds().first;
            pt(1) = ((x1Bounds().second - x1Bounds().first)/siz.second)*(double(cell.second) - 0.5) + x1Bounds().first;
            return pt;
        }
        amp::DenseArray2D<bool>& getdArr(){return dArr;};
        inline const amp::DenseArray2D<bool>& getdArr() const {return dArr;};
        
    private:
    amp::DenseArray2D<bool> dArr;
};

class MyCSpaceCtor: public amp::GridCSpace2DConstructor{
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override{
            std::unique_ptr<MyGridCSpace> ptr(new MyGridCSpace(250,250,0,2*M_PI,0,2*M_PI));
            MyLinkManipulator mani(manipulator.getBaseLocation(),manipulator.getLinkLengths());
            ptr->makeCSpace(mani, env);
            return ptr;
        }
};