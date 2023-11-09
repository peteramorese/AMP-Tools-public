#include "AMPCore.h"
#include "hw/HW7.h"
#include "CSpaceConstructor.h"
#include "HelpfulClass.h"

using namespace amp;
using std::vector, Eigen::VectorXd, std::pair;
using Regions = vector<vector<Edge>>;

class MyCentralChecker : amp::ConfigurationSpace {
    public:
        MyCentralChecker(const amp::MultiAgentProblem2D& problem) :
            amp::ConfigurationSpace(VectorXd(), VectorXd()), problem(problem) {
                regions = findRegions(findEdges(problem.obstacles));
                for (const CircularAgentProperties& agent : problem.agent_properties) {
                    radii.push_back(agent.radius * 1.1); 
                }
            }

        virtual bool inCollision(const Eigen::VectorXd& state) const override;
    private:
        MultiAgentProblem2D problem;
        vector<Regions> regions;
        vector<double> radii;
};

class MyGenericRRT {
    public:
        MyGenericRRT(int n, double r, double p, vector<std::pair<double, double>> limits)
        : n(n), r(r), p(p), limits(limits) {}
        amp::Path plan(const VectorXd& init_state, const VectorXd& goal_state, const MyCentralChecker& collision_checker); 
        Eigen::VectorXd getRandomPoint();
        void connectNieghbors(const vector<amp::Obstacle2D> obstacles);
        pair<int, Eigen::VectorXd> findNearest(const Eigen::VectorXd& point, const MyCentralChecker& collision_checker);
    private:
        int n, m;
        double r, p;
        bool smooth;
        std::map<uint32_t, VectorXd> points;
        std::map<uint32_t, uint32_t> parents;
        vector<std::pair<double, double>> limits;
};