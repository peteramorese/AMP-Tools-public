#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include <Eigen/LU>
#include <queue>
#include <algorithm>
#include <cmath>

using Node = uint32_t;

class checkPath {
    public:     
        bool checkXY(const double x, const double y, const amp::Environment2D obs){
            Eigen::Vector2d XY(x,y);
            switch(checkMode){
                case 0:
                    return pointCollision2D(XY,obs);
                    break;
                case 1:
                    return diskCollision2D(XY,tempAgent,obs);
                    break;
                default:
                    return pointCollision2D(XY,obs);
            }
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
        bool circleLineEval(const Eigen::Vector2d state, double radius, const Eigen::Vector2d p1, const Eigen::Vector2d p2){
            double a = p2(1) - p1(1); //delta y
            double b = p1(0) - p2(0); //-delta x
            double c = p2(1)*p1(0) - p2(0)*p1(1); // y2*x1 - x2*y1
            // Based on Erich Hartmann: Geometry and Algorithms for COMPUTER AIDED DESIGN. Lecture notes, Technische Universität Darmstadt, October 2003, p. 17
            double cPrime = c - a*state(0) - b*state(1);
            double discriminant = pow(radius,2)*(pow(a,2) + pow(b,2)) - pow(cPrime,2);
            if(discriminant > 0){
                // Check if secant line is within line segment (edge)
                Eigen::Vector2d temp1;
                Eigen::Vector2d temp2;
                temp1(0) = (a*cPrime + b*sqrt(discriminant))/(pow(a,2) + pow(b,2)) + state(0);
                temp2(0) = (a*cPrime - b*sqrt(discriminant))/(pow(a,2) + pow(b,2)) + state(0);

                temp1(1) = (b*cPrime - a*sqrt(discriminant))/(pow(a,2) + pow(b,2)) + state(1);
                temp2(1) = (b*cPrime + a*sqrt(discriminant))/(pow(a,2) + pow(b,2)) + state(1);

                return evalTU(getT(temp1,temp2,p1,p2),getU(temp1,temp2,p1,p2));
            }
            else{
                return false;
            }
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
        bool diskCollision2D(const Eigen::Vector2d state, amp::CircularAgentProperties& agent, const amp::Environment2D& obs){
            if(pointCollision2D(state,obs)){
                return true;
            }
            else{
                // Determine if disk intersects obstacle edge
                bool hit = false;
                for(auto Ob : obs.obstacles){
                        int numV = Ob.verticesCCW().size();
                        for(int j = 0; j < numV; j++){
                            if(circleLineEval(state, agent.radius, Ob.verticesCCW()[j], Ob.verticesCCW()[(j + 1) % numV])){
                                return true;
                            }
                        }
                }
                return false;
            }
        }
        bool multDiskCollision2D(const Eigen::VectorXd state, const Eigen::VectorXd next, const amp::MultiAgentProblem2D& problem){
            Eigen::Vector2d testState;
            Eigen::Vector2d testNext;
            for(int j = 0; j < 2*problem.numAgents(); j += 2){
                testState(0) = state(j);
                testState(1) = state(j + 1);
                testNext(0) = next(j);
                testNext(1) = next(j + 1);
                //Check for obstacle collisions TODO: FIX!!!
                if(lineCollision2D(testState, testNext, problem)){
                    return true;
                }
                //Check for robot-to-robot collisions
                for(int k = 0; k < 2*problem.numAgents(); k += 2){
                    if(j != k){
                        Eigen::Vector2d testState2(state(k),state(k + 1));
                        Eigen::Vector2d testNext2(next(k),next(k + 1));
                        if(evalTU(getT(testState,testNext,testState2,testNext2),getU(testState,testNext,testState2,testNext2))){
                            return true;
                        }
                    }
                }
            }
            return false;
        }
        amp::CircularAgentProperties& getAgent(){return tempAgent;};
        int& getW(){return checkMode;};
    private:
        amp::CircularAgentProperties tempAgent;
        int checkMode = 0;
};

class MyConfigurationSpace : public amp::ConfigurationSpace {

    virtual bool inCollision(const Eigen::VectorXd& cspace_state) const override{


        return true;
    }


};

class MyAStarAlgoShared : public amp::AStar {
    public:
        //node struct
        struct NodeStr{
            Node idx;
            Node back;
            double cost;
            NodeStr(Node i, Node b, double c){
                idx = i;
                back = b;
                cost = c;
            }
        };

        int inC(const std::vector<NodeStr>& C, Node idx){
            // Check if node is in closed list
            for(int j = 0; j < C.size(); j++){
                if(C[j].idx == idx){
                    return j;
                }
            }
            return -1;
        }

        int inO(const std::vector<NodeStr>& O, Node idx){
            // return index in O if node is in priority queue, or return -1 if node is not
            for(int j = 0; j < O.size(); j++){
                if(O[j].idx == idx){
                    return j;
                }
            }
            return -1;
        }

        struct compNodeStr{
            bool operator()(const NodeStr& a,const NodeStr& b) const{
                return a.cost > b.cost;
            }
        };

        // struct orderNodeStr{
        //     bool operator()(const NodeStr& a,const NodeStr& b) const{
        //         return a.ord < b.ord;
        //     }
        // };

        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {

            std::vector<NodeStr> O; //Priority Queue
            std::vector<NodeStr> C; //Processed Nodes
            GraphSearchResult GSR;
            int numIts = 0;
            NodeStr nBest(problem.init_node,problem.init_node,heuristic(problem.init_node));
            O.push_back(nBest);
            while(O.size() > 0){
                numIts++;
                // get smallest distance node
                std::pop_heap(O.begin(), O.end(), compNodeStr());
                nBest = O.back();
                // LOG("Popping node: (" << nBest.idx << "," << nBest.back << "," << nBest.cost << ")");
                O.pop_back();
                C.push_back(nBest);
                if(nBest.idx != problem.goal_node){
                    // add neighbors to queue
                    for(int j = 0; j < problem.graph->children(nBest.idx).size(); j++){
                        if(inC(C,problem.graph->children(nBest.idx)[j]) == -1){
                            Node nbrIdx = problem.graph->children(nBest.idx)[j];
                            double edge = problem.graph->outgoingEdges(nBest.idx)[j];
                            NodeStr nbr(nbrIdx, nBest.idx, nBest.cost + edge + heuristic(nbrIdx) - heuristic(nBest.idx));
                            int oIdx = inO(O, nbr.idx);
                            if(oIdx == -1){
                                // LOG("Adding child: (" << nbr.idx << "," << nbr.back << "," << nbr.cost << ")");
                                O.push_back(nbr);
                                std::push_heap(O.begin(), O.end(), compNodeStr());
                            }
                            else if(nbr.cost < O[oIdx].cost){
                                // LOG("Update child: (" << nbr.idx << "," << nbr.back << "," << nbr.cost << ")");
                                O[oIdx] = nbr;
                                std::make_heap(O.begin(), O.end(), compNodeStr());
                            }
                        }
                    }
                }
                else{
                    O.clear();
                    // LOG("Goal found at: " << nBest.idx << " back: " << nBest.back << " cost: " << nBest.cost);
                    GSR.path_cost = (nBest.cost - heuristic(nBest.idx));
                    while(nBest.idx != problem.init_node){
                        GSR.node_path.push_front(nBest.idx);
                        nBest = C[inC(C,nBest.back)];
                    }
                    GSR.node_path.push_front(problem.init_node);
                    // LOG("Found Path with cost " << GSR.path_cost << " after " << numIts << " iterations:");
                    // for(auto ele : GSR.node_path){
                    //     std::cout << ele << " -> ";
                    // }
                    // std::cout << "done :)" << std::endl;
                    GSR.success = true;
                }


            }

            return GSR;
        }
};

class MyGoalBiasRRTND : public amp::GoalBiasRRT2D {
    public:


        struct sampleS{
            Eigen::VectorXd xy;
            int back = 0;
            sampleS(const Eigen::VectorXd& inXY, int inBack){
                xy = inXY;
                back = inBack;
            }
        };

        /// @brief Solve a motion planning problem. Create a derived class and override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override{
            amp::Path2D path;
            return path;
        };

        amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem);

        std::map<Node, Eigen::Vector2d> makeMap(std::vector<Eigen::Vector2d> samples);

        int& getN(){return numIterations;};
        int& getT(){return time;};
        double& getG(){return goalBiasP;};
        double& getS(){return stepSize;};
        double& getE(){return eps;};
        bool& getW(){return web_slinger;};
    private:
        int time = 0;
        int numIterations = 20000;
        double goalBiasP = 0.1;
        double stepSize = 1;
        double eps = 1;
        bool web_slinger = false;
};