#pragma once

#include "AMPCore.h"
#include <Eigen/LU>
class checkPath {
    public:
        void hereIsAMethod();
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
        bool pointCollision2D(const Eigen::Vector2d state, const amp::Environment2D& obs){
            bool hit = false;
            Eigen::Vector2d nextVertex;
            Eigen::Vector2d currentVertex;
            double d;
            for(auto Ob : obs.obstacles){
                if(!hit){
                    hit = false;
                    int pos = 0;
                    int neg = 0;
                    for(int j = 0; j < Ob.verticesCCW().size(); j++){
                        nextVertex = Ob.verticesCCW()[(j + 1)%Ob.verticesCCW().size()];
                        currentVertex = Ob.verticesCCW()[j];
                        d = (state(0) - currentVertex(0))*(nextVertex(1) - currentVertex(1)) - (state(1) - currentVertex(1))*(nextVertex(0) - currentVertex(0));

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
            return hit;
        }
        bool lineCollision2D(const Eigen::Vector2d state0, const Eigen::Vector2d state1, const amp::Environment2D& obs){
            bool hit = false;
            for(auto Ob : obs.obstacles){
                if(!hit){
                    for(int j = 0; j < Ob.verticesCCW().size(); j++){
                        if(!hit){
                            hit = checkHit(state0,state1,Ob,j);
                        }
                    }
                }
            }
            return hit;
        }
};

class MyConfigurationSpace : public amp::ConfigurationSpace {

    virtual bool inCollision(const Eigen::VectorXd& cspace_state) const override{


        return true;
    }


};