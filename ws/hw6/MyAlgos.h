#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <Eigen/LU>
#include <queue>

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


class MyPointWFAlgo : public MyWaveFrontAlgorithm, amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override {
            return std::make_unique<MyGridCSpace2D>(1, 1, 0.0, 1.0, 0.0, 1.0);
        }

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            return amp::Path2D();
        }

        // // You need to implement here
        // virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
        //     return amp::Path2D();
        // }
};

class MyManipWFAlgo : public MyWaveFrontAlgorithm, public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>()) {}

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
                                if((i >= 0 && j >= 0 && i < grid_cspace.size().first && j < grid_cspace.size().second) && WVArr(i,j) == 0){
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
};

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {
            return GraphSearchResult();
        }
};