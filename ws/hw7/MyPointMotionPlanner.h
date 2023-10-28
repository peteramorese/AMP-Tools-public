#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include <Eigen/LU>
#include "HelpfulClass.h"


struct MySearchHeuristic : public amp::SearchHeuristic {
	/// @brief Default heuristic that just returns L2 norm distance to goal.
	/// @param node Node to get the heuristic value h(node) for. 
	/// @return Heuristic value
    Eigen::Vector2d goal;
    std::vector<Eigen::Vector2d> samples;
    MySearchHeuristic(const Eigen::Vector2d& goal_, std::vector<Eigen::Vector2d>& samples_){
        goal = goal_;
        samples = samples_;
    }
	virtual double operator()(amp::Node node) const {return (goal - samples[node]).norm();}
};

// /// @brief Derive this class and implement your algorithm in the `plan` method. 
// class MyPRM : public HW7::PointMotionPlanner2D {
//     public:
//         /// @brief Solve a motion planning problem. Create a derived class and override this method
//         //virtual amp::Path2D plan(const amp::Problem2D& problem) = 0;

//         virtual ~PRM() {}
// };
class MyPRM : public amp::PRM2D {
    public:
        /// @brief Solve a motion planning problem. Create a derived class and override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override{
            // TODO:
            //1. Sample numSamples times to fill vector of Eigen samples (use rand() % x_max + x_min)
            //1.1. Check that each sample is valid before adding to vector (point polygon check)
            //2. Connect every sample within r radius with straight line (make graph?)
            //2.1. Check that each edge doesn't collide (line polygon check)
            //3. Search graph from start to goal, return node nums, convert to appropriate Eigen Vectors
            amp::Path2D path;
            std::vector<Eigen::Vector2d> samples;
            samples.push_back(problem.q_init);
            samples.push_back(problem.q_goal);
            auto graph = std::make_shared<amp::Graph<double>>();
            Eigen::Vector2d temp;
            int id = 2;
            checkPath c;
            for(int j = 0; j < numSamples; j++){
                temp(0) = amp::RNG::randd(problem.x_min, problem.x_max);
                temp(1) = amp::RNG::randd(problem.y_min, problem.y_max);
                if(!c.pointCollision2D(temp,problem)){
                    for(int k = 0; k < samples.size(); k++){
                        if((samples[k] - temp).norm() <= radius && !c.lineCollision2D(temp, samples[k], problem)){
                            graph->connect(k,id,(samples[k] - temp).norm());
                            graph->connect(id,k,(samples[k] - temp).norm());
                        }
                    }
                    samples.push_back(temp);
                    id++;
                }
            }
            MySearchHeuristic sh(problem.q_goal,samples);
            MyAStarAlgo As;
            amp::ShortestPathProblem spp;
            spp.graph = graph;
            spp.init_node = 0;
            spp.goal_node = 1;
            // amp::GraphSearchResult Ares =  As.search(spp,sh);

            for(auto idx : As.search(spp,sh).node_path){
                path.waypoints.push_back(samples[idx]);
            }

            return path;
        }

        amp::Path planND(const amp::Problem2D& problem, const amp::ConfigurationSpace& checker){
            // TODO:
            //1. Sample numSamples times to fill vector of Eigen samples (use rand() % x_max + x_min)
            //1.1. Check that each sample is valid before adding to vector (point polygon check)
            //2. Connect every sample within r radius with straight line (make graph?)
            //2.1. Check that each edge doesn't collide (line polygon check)
            //3. Search graph from start to goal, return node nums, convert to appropriate Eigen Vectors
            return amp::Path();
        }



        // virtual ~PRM() {}
        int& getN(){return numSamples;};
        double& getR(){return radius;};
    private:
        int numSamples = 100;
        double radius = 1;
};

// /// @brief Derive this class and implement your algorithm in the `plan` method. 
// class MyGoalBiasRRT : public HW7::PointMotionPlanner2D {
//     public:
//         /// @brief Solve a motion planning problem. Create a derived class and override this method
//         //virtual amp::Path2D plan(const amp::Problem2D& problem) = 0;

//         virtual ~GoalBiasRRT() {}
// };

class MyGoalBiasRRT : public amp::PointMotionPlanner2D {
    public:
        /// @brief Solve a motion planning problem. Create a derived class and override this method
        amp::Path2D plan(const amp::Problem2D& problem){

            return amp::Path2D();
        }

        // virtual ~GoalBiasRRT() {}
};
