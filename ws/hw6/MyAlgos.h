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
                    std::size_t tempNbrs[] = {cell.first - 1, cell.second, cell.first + 1, cell.second, cell.first, cell.second - 1, cell.first, cell.second + 1};
                    for(int n = 0; n < 7; n++){
                        std::size_t i = tempNbrs[n];
                        std::size_t j = tempNbrs[n + 1];
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
            return amp::Path2D();
        }     

        /*****************************************/
    };


class MyPointWFAlgo : public MyWaveFrontAlgorithm, amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override {
            return std::make_unique<MyGridCSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
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

class MyManipWFAlgo : public MyWaveFrontAlgorithm, amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {}

        // You can have custom ctor params for all of these classes
        MyManipWFAlgo(const std::string& beep) 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {LOG("construcing... " << beep);}

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::ManipulatorTrajectory2Link plan(const amp::LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
            return amp::ManipulatorTrajectory2Link();
        }
        
        // // You need to implement here
        // virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
        //     return amp::Path2D();
        // }
};

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {
            return GraphSearchResult();
        }
};