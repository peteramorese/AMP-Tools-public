#pragma once

#include "AMPCore.h"
#include <Eigen/LU>
#include <queue>
#include <algorithm>
#include <cmath>

using Node = uint32_t;

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

class MyAStarAlgo : public amp::AStar {
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