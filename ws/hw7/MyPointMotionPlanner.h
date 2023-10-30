#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include <Eigen/LU>
#include "HelpfulClass.h"
#include <chrono>

using Node = uint32_t;

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
        MyPRM(){}
        MyPRM(int nS, double rad, bool sM){
            numSamples = nS;
            radius = rad;
            smooth = sM;
        }
        /// @brief Solve a motion planning problem. Create a derived class and override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override{
            // TODO:
            //1. Sample numSamples times to fill vector of Eigen samples (use rand() % x_max + x_min)
            //1.1. Check that each sample is valid before adding to vector (point polygon check)
            //2. Connect every sample within r radius with straight line (make graph?)
            //2.1. Check that each edge doesn't collide (line polygon check)
            //3. Search graph from start to goal, return node nums, convert to appropriate Eigen Vectors
            auto start = std::chrono::high_resolution_clock::now();
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
            
            // auto stop = std::chrono::high_resolution_clock::now();
            // auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
            // LOG("sampling time: " << duration.count());
            // LOG("# Valid samples: " << samples.size());
            // LOG("# Valid nodes: " << graph->nodes().size());
            MySearchHeuristic sh(problem.q_goal,samples);
            MyAStarAlgo As;
            amp::ShortestPathProblem spp;
            spp.graph = graph;
            spp.init_node = 0;
            spp.goal_node = 1;
            amp::AStar::GraphSearchResult searchResult =  As.search(spp,sh);

            if(searchResult.success){
                for(auto idx : searchResult.node_path){
                    path.waypoints.push_back(samples[idx]);
                }
                if(smooth){
                    pathSmoother(problem, path);
                }
            }
            else{
                // LOG("Can't find a path :(");
                path.waypoints.push_back(problem.q_init);
                path.waypoints.push_back(problem.q_goal);
            }
            if(web_slinger && amp::HW7::check(path,problem,false)){
                // makeMap(samples);
                amp::Visualizer::makeFigure(problem, path);
                amp::Visualizer::makeFigure(problem, *graph.get(), makeMap(samples));
                amp::Visualizer::showFigures();
                LOG("path length: " << path.length());
            }
            // LOG("length of nodePath: " << searchResult.node_path.size());
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
            time = duration.count();
            return path;
            
        }

        void pathSmoother(const amp::Problem2D& problem, amp::Path2D& path){
            checkPath c;
            int idx1 = 0;
            int idx2 = 1;
            int initSize = path.waypoints.size();
            for(int j = 0; j < 10*initSize; j ++){
                if(path.waypoints.size() > 2){
                    do{
                    idx1 = amp::RNG::randi(0,path.waypoints.size());
                    idx2 = amp::RNG::randi(0,path.waypoints.size());
                    }while(idx1 >= idx2 || idx2 - idx1 == 1);
                    if(!c.lineCollision2D(path.waypoints[idx1], path.waypoints[idx2], problem)){
                        // LOG("removing idx between " << idx1 << " and " << idx2);
                        path.waypoints.erase(path.waypoints.begin() + idx1 + 1, path.waypoints.begin() + idx2);
                    }
                }
            }
        }
        std::map<Node, Eigen::Vector2d> makeMap(std::vector<Eigen::Vector2d> samples){
            std::map<Node, Eigen::Vector2d> mapOfSamples;
            for (Node j = 0; j < samples.size(); j++){ 
                mapOfSamples.insert({j, samples[j]}); 
            }
            return mapOfSamples;
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
        int& getT(){return time;};
        bool& getS(){return smooth;};
        bool& getW(){return web_slinger;};
    private:
        int time = 0;
        int numSamples = 300;
        double radius = 4;
        bool smooth = false;
        bool web_slinger = false;
};


// /// @brief Derive this class and implement your algorithm in the `plan` method. 
class MyGoalBiasRRT : public amp::GoalBiasRRT2D {
    public:


        struct sampleS{
            Eigen::Vector2d xy;
            int back = 0;
            sampleS(const Eigen::Vector2d& inXY, int inBack){
                xy = inXY;
                back = inBack;
            }
        };

        /// @brief Solve a motion planning problem. Create a derived class and override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override{
            amp::Path2D path;
            std::vector<sampleS> samples;
            samples.push_back(sampleS(problem.q_init,-1));
            Eigen::Vector2d q_rand;
            bool soln = false;
            int minID = 0;
            double tempMin = 0;
            checkPath c;
            int steps = 0;
            while(!soln && steps < numIterations){
                //Generate q_rand
                if(amp::RNG::randi(0,100) < goalBiasP*100){
                    q_rand(0) = amp::RNG::randd(problem.x_min, problem.x_max);
                    q_rand(1) = amp::RNG::randd(problem.y_min, problem.y_max);
                }
                else{
                    q_rand = problem.q_goal;
                }
                //Find closest node in tree to q_rand
                minID = 0;
                tempMin = (samples[0].xy - q_rand).norm();
                for(int k = 1; k < samples.size(); k++){
                    if((samples[k].xy - q_rand).norm() < tempMin){
                        tempMin = (samples[k].xy - q_rand).norm();
                        minID = k;
                    }
                }
                q_rand = ((1 - stepSize/tempMin)*samples[minID].xy + (stepSize/tempMin)*q_rand);
                if(!c.lineCollision2D(q_rand, samples[minID].xy, problem)){
                    sampleS q_randS(q_rand,minID);
                    samples.push_back(q_randS);
                    if((q_rand - problem.q_goal).norm() < eps){
                        soln = true;
                    }
                }
                steps++;
            }
            std::list<Eigen::Vector2d> l;
            if(soln){
                sampleS testS(problem.q_goal,samples.size() - 1);
                while(testS.back != -1){
                    l.push_front(testS.xy);
                    testS = samples[testS.back];
                }
                l.push_front(problem.q_init);
                path.waypoints = { std::begin(l), std::end(l) };
            }
            else{
                if(steps == numIterations){
                    LOG("Ran outta steps buddy :(");
                }
                LOG("Couldn't find path :(");
                path.waypoints.push_back(problem.q_init);
                path.waypoints.push_back(problem.q_goal);
            }

            return path;


        };

        int& getN(){return numIterations;};
        double& getG(){return goalBiasP;};
        double& getS(){return stepSize;};
        double& getE(){return eps;};
    private:
        int numIterations = 20000;
        double goalBiasP = 0.1;
        double stepSize = 1;
        double eps = 1;
};

