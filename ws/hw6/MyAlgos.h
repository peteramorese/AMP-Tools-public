#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>
#include <queue>
#include <algorithm>
#include "MyConfigurationSpace.h"

using Node = uint32_t;

class MyWaveFrontAlgorithm: public amp::WaveFrontAlgorithm {
    public:
        /******* User Implemented Methods ********/

        /// @brief Return a non-colliding path through a grid C-space using the WaveFront algorithm. Override this method and implement your WaveFront planner
        /// @param grid_cspace Your grid discretization C-space from HW4. 
        /// NOTE: For Exercise 1), you will need to manually construct the discretization for HW2 Exercise2.
        /// NOTE: For Exercise 2), You can use 
        /// @return A path inside the C-space. Your WaveFront planner will find a sequence of cells. This path should be a sequence of representative points for each cell in the sequence.
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override{
            //0. Create dense array for wave front cell values
            std::pair<std::size_t, std::size_t> siz = grid_cspace.size();
            amp::DenseArray2D<int> WVArr(siz.first,siz.second,0);

            //1. Get cell for q_goal and assign value of 2
            std::pair<std::size_t, std::size_t> cell = grid_cspace.getCellFromPoint(q_goal(0),q_goal(1));
            WVArr(cell.first,cell.second) = 2;

            std::queue<std::pair<std::size_t, std::size_t>> Queue;
            //2. Iterate through cells connected to q_goal and not colliding according to GridSpace2D and add 1
            //  Create queue array: Add q_goal initially.
                Queue.push(cell);
                while(Queue.size() > 0){
                    //  a) pop top of queue
                    cell = Queue.front();
                    Queue.pop();
                    //  b) check (i+1,j),(i-1,j),(i,j+1),(i,j-1)
                    //      i)  if idx collides, set value in dense array to 1
                    //      ii) else if value in dense array is zero, set value to denseArray(i,j) + 1 and add to back of queue
                    for(int m = -1; m < 2; m++){
                        for(int n = -1; n < 2; n++){
                            if(m != n){
                                std::size_t i = cell.first + m;
                                std::size_t j = cell.second + n;
                                if((i >= 0 && j >= 0 && i <= grid_cspace.size().first && j <= grid_cspace.size().second) && WVArr(i,j) == 0){
                                    //check if (i,j) collides
                                    if(grid_cspace(i,j)){
                                        WVArr(i,j) = 1;
                                    }
                                    else{
                                        WVArr(i,j) =  WVArr(cell.first,cell.second) + 1;
                                        std::pair<std::size_t, std::size_t> nbr(i,j);
                                        Queue.push(nbr);
                                    }
                                }
                            }
                        }
                    }
                }
            //3. Make plan based on filled wavefront thing
            amp::Path2D path;
            Eigen::Vector2d pt(0,0);
            cell = grid_cspace.getCellFromPoint(q_init(0),q_init(1));
            path.waypoints.push_back(q_init);
            //Push initial point cell centerpoint
            pt(0) = ((grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/siz.first)*(double(cell.first) - 0.5) + grid_cspace.x0Bounds().first;
            pt(1) = ((grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/siz.second)*(double(cell.second) - 0.5) + grid_cspace.x1Bounds().first;
            path.waypoints.push_back(pt);

            std::pair<std::size_t, std::size_t> next = cell;
            while(WVArr(cell.first,cell.second) != 2){
                for(int m = -1; m < 2; m++){
                    for(int n = -1; n < 2; n++){
                        if(m != n){
                            std::size_t i = cell.first + m;
                            std::size_t j = cell.second + n;
                            if(WVArr(cell.first + m,cell.second + n) < WVArr(next.first,next.second)){
                                next.first = cell.first + m;
                                next.second = cell.second + n;
                            }
                        }
                    }
                }
                // push centerpoint of next cell and move cell to next
                cell = next;
                pt(0) = ((grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/siz.first)*(double(cell.first) - 0.5) + grid_cspace.x0Bounds().first;
                pt(1) = ((grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/siz.second)*(double(cell.second) - 0.5) + grid_cspace.x1Bounds().first;
                path.waypoints.push_back(pt);

            }
            // move from centerpoint of goal cell to goal :)
            path.waypoints.push_back(q_goal);
            return path;
        }

        /*****************************************/
    };


class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override {
            MyGridCSpace2DConstructor cons;
            std::pair<double, double> x0_bounds(environment.x_min,environment.x_max);
            std::pair<double, double> x1_bounds(environment.y_min,environment.y_max);
            cons.getGridWidth() = 0.25;
            cons.getX0_bounds() = x0_bounds;
            cons.getX1_bounds() = x1_bounds;
            return cons.construct(environment);
        }

        // This is just to get grade to work, you DO NOT need to override this method
        // virtual amp::Path2D plan(const amp::Problem2D& problem) override {
        //     return amp::Path2D();
        // }

        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override{
            //0. Create dense array for wave front cell values and pad obstacles
            std::pair<std::size_t, std::size_t> siz = grid_cspace.size();
            amp::DenseArray2D<int> WVArr(siz.first,siz.second,0);
            for(int i = 0; i < siz.first; i++){
                for(int j = 0; j < siz.second; j++){
                    double x0 = ((grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/siz.first)*(i + 0.5) + grid_cspace.x0Bounds().first;
                    double x1 = ((grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/siz.second)*(j + 0.5) + grid_cspace.x1Bounds().first;
                    if(grid_cspace.inCollision(x0,x1)){
                        if(i != 0){WVArr(i-1,j) = 1;}
                        if(i != grid_cspace.size().first - 1){WVArr(i+1,j) = 1;}
                        if(j != 0 ){WVArr(i,j-1) = 1;}
                        if(j != grid_cspace.size().second - 1){WVArr(i,j+1) = 1;}
                    }
                }
            }
            WVArr(grid_cspace.getCellFromPoint(q_init(0),q_init(1)).first,grid_cspace.getCellFromPoint(q_init(0),q_init(1)).second) = 0;
            //1. Get cell for q_goal and assign value of 2
            std::pair<std::size_t, std::size_t> cell = grid_cspace.getCellFromPoint(q_goal(0),q_goal(1));
            WVArr(cell.first,cell.second) = 2;
            
            std::queue<std::pair<std::size_t, std::size_t>> Queue;
            //2. Iterate through cells connected to q_goal and not colliding according to GridSpace2D and add 1
            //  Create queue array: Add q_goal initially.
            Queue.push(cell);
            while(Queue.size() > 0){
                //  a) pop top of queue
                cell = Queue.front();
                Queue.pop();
                //  b) check (i+1,j),(i-1,j),(i,j+1),(i,j-1)
                //      i)  if idx collides, set value in dense array to 1
                //      ii) else if value in dense array is zero, set value to denseArray(i,j) + 1 and add to back of queue
                for(int m = -1; m < 2; m++){
                    for(int n = -1; n < 2; n++){
                        if((m == 0 || n == 0) && m != n){
                            std::size_t i = cell.first + m;
                            std::size_t j = cell.second + n;

                            if((i >= 0 && j >= 0 && i < grid_cspace.size().first && j < grid_cspace.size().second) && WVArr(i,j) == 0){

                                double x0 = ((grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/siz.first)*(i + 0.5) + grid_cspace.x0Bounds().first;
                                double x1 = ((grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/siz.second)*(j + 0.5) + grid_cspace.x1Bounds().first;
                                if(grid_cspace.inCollision(x0,x1)){
                                    WVArr(i,j) = 1;
                                    LOG("this grid had an unfound collision!: " << i << " , " << j);
                                    // if(i != 0 && WVArr(i-1,j) != 0){WVArr(i-1,j) = 1;}
                                    // if(i != grid_cspace.size().first - 1 && WVArr(i+1,j) != 0){WVArr(i+1,j) = 1;}
                                    // if(j != 0 && WVArr(i,j-1) != 0){WVArr(i,j-1) = 1;}
                                    // if(j != grid_cspace.size().second - 1 && WVArr(i,j+1) != 0){WVArr(i,j+1) = 1;}
                                    
                                    // std::pair<std::size_t, std::size_t> nbr(i,j);
                                }
                                else{
                                    WVArr(i,j) =  WVArr(cell.first,cell.second) + 1;
                                    std::pair<std::size_t, std::size_t> nbr(i,j);
                                    Queue.push(nbr);
                                }
                            }
                        }
                    }
                }
            }
            


            //3. Make plan based on filled wavefront thing
            //Get lookahead
            amp::Path2D path;
            Eigen::Vector2d pt(0,0);
            cell = grid_cspace.getCellFromPoint(q_init(0),q_init(1));
            path.waypoints.push_back(q_init);

            std::pair<std::size_t, std::size_t> next = cell;
            while(WVArr(cell.first,cell.second) != 2){
                for(int m = -1; m < 2; m++){
                    for(int n = -1; n < 2; n++){
                        if((m == 0 || n == 0) && m != n){
                            std::size_t i = cell.first + m;
                            std::size_t j = cell.second + n;

                            if((i >= 0 && j >= 0 && i < grid_cspace.size().first && j < grid_cspace.size().second)){
                                if((WVArr(i,j) < WVArr(next.first,next.second)) && WVArr(i,j) != 1){
                                    next.first = i;
                                    next.second = j;
                                }
                            }
                            
                        }
                    }
                }
                if(cell == next){
                    //NO PATH AVAILABLE!
                    pt(0) = ((grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/siz.first)*(double(cell.first) + 0.5) + grid_cspace.x0Bounds().first;
                    pt(1) = ((grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/siz.second)*(double(cell.second) + 0.5) + grid_cspace.x1Bounds().first;
                    std::cout << "failure! could not find path from " << q_init << " to " << q_goal << " , cell val and center: " << WVArr(next.first,next.second) << " , " << pt << std::endl;
                    return path;
                }
                // push centerpoint of next cell and move cell to next
                cell = next;
                pt(0) = ((grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/siz.first)*(double(cell.first) + 0.5) + grid_cspace.x0Bounds().first;
                pt(1) = ((grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/siz.second)*(double(cell.second) + 0.5) + grid_cspace.x1Bounds().first;
                
                if(WVArr(cell.first,cell.second) != 2){
                    path.waypoints.push_back(pt);
                }

            }
            // move from centerpoint of goal cell to goal :)
            path.waypoints.push_back(q_goal);
            return path;
        }
};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // // Default ctor
        MyManipWFAlgo(MyGridCSpace2DConstructor& constructor)
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>(constructor)) {

            }

        // // You can have custom ctor params for all of these classes
        // MyManipWFAlgo(const std::string& beep) 
        //     : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {LOG("construcing... " << beep);}

        // // This is just to get grade to work, you DO NOT need to override this method
        // virtual amp::ManipulatorTrajectory2Link plan(const amp::LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
        //     return amp::ManipulatorTrajectory2Link();
        // }
        
        // // // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override{
            //0. Create dense array for wave front cell values
            std::pair<std::size_t, std::size_t> siz = grid_cspace.size();
            amp::DenseArray2D<int> WVArr(siz.first,siz.second,0);
            // for(int i = 0; i < siz.first; i++){
            //     for(int j = 0; j < siz.second; j++){
            //         double x0 = ((grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/siz.first)*(i + 0.5) + grid_cspace.x0Bounds().first;
            //         double x1 = ((grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/siz.second)*(j + 0.5) + grid_cspace.x1Bounds().first;
            //         if(grid_cspace.inCollision(x0,x1)){
            //             if(i != 0){WVArr(i-1,j) = 1;}
            //             if(i != grid_cspace.size().first - 1){WVArr(i+1,j) = 1;}
            //             if(j != 0 ){WVArr(i,j-1) = 1;}
            //             if(j != grid_cspace.size().second - 1){WVArr(i,j+1) = 1;}
            //         }
            //     }
            // }
            //1. Get cell for q_goal and assign value of 2
            std::pair<std::size_t, std::size_t> cell = grid_cspace.getCellFromPoint(q_goal(0),q_goal(1));
            if(int(cell.first) < 0){
                cell.first = grid_cspace.size().first - 1;
            }
            else if(cell.first >= grid_cspace.size().first){
                cell.first = 0;
            }
            if(int(cell.second) < 0){
                cell.second = grid_cspace.size().second - 1;
            }
            else if(cell.second >= grid_cspace.size().second){
                cell.second = 0;
            }
            
            WVArr(cell.first,cell.second) = 2;
            // std::cout << "cell.first " << cell.first << " cell.second " << cell.second << std::endl;
            std::queue<std::pair<std::size_t, std::size_t>> Queue;
            //2. Iterate through cells connected to q_goal and not colliding according to GridSpace2D and add 1
            //  Create queue array: Add q_goal initially.
                Queue.push(cell);
                while(Queue.size() > 0){
                    //  a) pop top of queue
                    cell = Queue.front();
                    Queue.pop();
                    //  b) check (i+1,j),(i-1,j),(i,j+1),(i,j-1)
                    //      i)  if idx collides, set value in dense array to 1
                    //      ii) else if value in dense array is zero, set value to denseArray(i,j) + 1 and add to back of queue
                    for(int m = -1; m < 2; m++){
                        for(int n = -1; n < 2; n++){
                            if((m == 0 || n == 0) && m != n){
                                std::size_t i = cell.first + m;
                                std::size_t j = cell.second + n;
                                // std::cout << "cell.first " << cell.first << " + " << m << " = " << i << std::endl;
                                if(m < 0 && cell.first == 0){
                                    // std::cout << "help1 "<< int(i) << std::endl;
                                    i = grid_cspace.size().first - 1;
                                }
                                else if(i >= grid_cspace.size().first){
                                    // std::cout << "help2 "<< int(i) << std::endl;
                                    i = 0;
                                }
                                if(n < 0 && cell.second == 0){
                                    // std::cout << "help3 "<< int(j) << std::endl;
                                    j = grid_cspace.size().second - 1;
                                }
                                else if(j >= grid_cspace.size().second){
                                    // std::cout << "help4 "<< int(j) << std::endl;
                                    j = 0;
                                }
                                // if((i >= 0 && j >= 0 && i < grid_cspace.size().first && j < grid_cspace.size().second) && WVArr(i,j) == 0){
                                if(WVArr(i,j) == 0){
                                    //check if (i,j) collides
                                    double x0 = ((grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/siz.first)*(i + 0.5) + grid_cspace.x0Bounds().first;
                                    double x1 = ((grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/siz.second)*(j + 0.5) + grid_cspace.x1Bounds().first;
                                    if(grid_cspace.inCollision(x0,x1)){
                                        WVArr(i,j) = 1;
                                        std::pair<std::size_t, std::size_t> nbr(i,j);
                                        // Queue.push(nbr);
                                    }
                                    else{
                                        WVArr(i,j) =  WVArr(cell.first,cell.second) + 1;
                                        std::pair<std::size_t, std::size_t> nbr(i,j);
                                        Queue.push(nbr);
                                    }
                                }
                            }
                        }
                    }
                }
            //3. Make plan based on filled wavefront thing
            //Get lookahead
            // LOG("Filled WF");
            amp::Path2D path;
            Eigen::Vector2d pt(0,0);
            cell = grid_cspace.getCellFromPoint(q_init(0),q_init(1));
            path.waypoints.push_back(q_init);
            //Push initial point cell centerpoint

            std::pair<std::size_t, std::size_t> next = cell;
            while(WVArr(cell.first,cell.second) != 2){
                for(int m = -1; m < 2; m++){
                    for(int n = -1; n < 2; n++){
                        if((m == 0 || n == 0) && m != n){
                            std::size_t i = cell.first + m;
                            std::size_t j = cell.second + n;
                            if(i < 0){
                                // std::cout << "help1 "<< i << std::endl;
                                i = grid_cspace.size().first - 1;
                            }
                            else if(i >= grid_cspace.size().first){
                                // std::cout << "help2 "<< i << std::endl;
                                i = 0;
                            }
                            if(j < 0){
                                // std::cout << "help3 "<< j << std::endl;
                                j = grid_cspace.size().second - 1;
                            }
                            else if(j >= grid_cspace.size().second){
                                // std::cout << "help4 "<< j << std::endl;
                                j = 0;
                            }

                            if((WVArr(i,j) < WVArr(next.first,next.second)) && WVArr(i,j) != 1){
                                next.first = i;
                                next.second = j;
                                // std::cout << "next: " << next.first << " , " << next.second << " val: " << WVArr(i,j) << std::endl;
                            }
                            
                        }
                    }
                }
                if(cell == next){
                    //NO PATH AVAILABLE!
                    LOG("failure! could not find path from " << q_init << " to " << q_goal);
                    path.waypoints.push_back(q_goal);
                    return path;
                }
                // push centerpoint of next cell and move cell to next
                cell = next;
                pt(0) = ((grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/siz.first)*(double(cell.first) + 0.5) + grid_cspace.x0Bounds().first;
                pt(1) = ((grid_cspace.x1Bounds().second - grid_cspace.x1Bounds().first)/siz.second)*(double(cell.second) + 0.5) + grid_cspace.x1Bounds().first;
                
                if(WVArr(cell.first,cell.second) != 2){
                    path.waypoints.push_back(pt);
                }

            }
            // move from centerpoint of goal cell to goal :)
            // LOG("Found a path :)");
            path.waypoints.push_back(q_goal);
            return path;
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
            NodeStr nBest(problem.init_node,problem.init_node,heuristic(problem.init_node));
            O.push_back(nBest);
            while(O.size() > 0){
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
                    // LOG("Found Path with cost " << GSR.path_cost << " :");
                    // for(auto ele : GSR.node_path){
                    //     std::cout <<  " -> " << ele;
                    // }
                    // std::cout << std::endl;
                    GSR.success = true;
                }


            }

            return GSR;
        }
};