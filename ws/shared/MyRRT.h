#include "AMPCore.h"
#include "hw/HW7.h"
#include "CSpaceConstructor.h"
#include "HelpfulClass.h"

using namespace amp;
using std::vector, Eigen::VectorXd, std::pair;
using Regions = vector<vector<Edge>>;

class MyCentralChecker : amp::ConfigurationSpace {
    public:
        MyCentralChecker(const amp::MultiAgentProblem2D& problem, bool central) :
            central(central), amp::ConfigurationSpace(VectorXd(), VectorXd()), problem(problem) {
                regions = findRegions(findEdges(problem.obstacles));
                for (const CircularAgentProperties& agent : problem.agent_properties) {
                    radii.push_back(agent.radius * 1.1); 
                }
            }

        virtual bool inCollision(const Eigen::VectorXd& state) const override;
        bool inCollisionCentral(const Eigen::VectorXd& state) const;
        bool inCollisionDecentral(const Eigen::VectorXd& state) const;
        bool checkWithPrior(const Eigen::Vector2d& state) const;
        bool checkK0condtion(const Eigen::Vector2d& state) const;
        bool avoidGoals(const Eigen::Vector2d& state) const;

        void addPath(const Path& path);
        pair<int, VectorXd> nearest;
        bool central;
    private:
        MultiAgentProblem2D problem;
        vector<Path> computedPaths;
        vector<Regions> regions;
        vector<double> radii;
};

// class MyDecentralChecker : amp::ConfigurationSpace {
//     public:
//         MyDecentralChecker(const amp::MultiAgentProblem2D& problem) :
//             amp::ConfigurationSpace(VectorXd(), VectorXd()), problem(problem) {
//                 regions = findRegions(findEdges(problem.obstacles));
//                 for (const CircularAgentProperties& agent : problem.agent_properties) {
//                     radii.push_back(agent.radius * 1.1); 
//                 }
//             }

//         virtual bool inCollision(const Eigen::VectorXd& state) const override;
//         bool checkWithPrior(const Eigen::Vector2d& state) const; 

//     private:
//         MultiAgentProblem2D problem;
//         vector<Regions> regions;
//         vector<double> radii;
// };


class MyGenericRRT {
    public:
        MyGenericRRT(int n, double r, double p, double eps, vector<std::pair<double, double>> limits)
        : n(n), r(r), p(p), eps(eps), limits(limits) {}
        amp::Path plan(const VectorXd& init_state, const VectorXd& goal_state, MyCentralChecker& collision_checker); 
        Eigen::VectorXd getRandomPoint();
        int findStepsToRoot(int node);
        pair<int, Eigen::VectorXd> findNearest(const Eigen::VectorXd& point, MyCentralChecker& collision_checker);
        int treeSize;
    private:
        int n, m;
        double r, p, eps;
        bool smooth;
        std::map<uint32_t, VectorXd> points;
        std::map<uint32_t, uint32_t> parents;
        vector<std::pair<double, double>> limits;
};