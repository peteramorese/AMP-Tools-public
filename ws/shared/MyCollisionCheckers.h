#include "AMPCore.h"
#include "CSpaceConstructor.h"
#include "HelpfulClass.h"

using std::vector, std::pair, Eigen::VectorXd;
using Regions = vector<vector<Edge>>;

class MyCentralChecker : amp::ConfigurationSpace {
    public:
        MyCentralChecker(const amp::MultiAgentProblem2D& problem, bool central) :
            central(central), amp::ConfigurationSpace(VectorXd(), VectorXd()), problem(problem) {
                regions = findRegions(findEdges(problem.obstacles));
                for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
                    radii.push_back(agent.radius * 1.1); 
                }
            }

        virtual bool inCollision(const Eigen::VectorXd& state) const override;
        bool inCollisionSingle(const Eigen::VectorXd& state) const;
        bool inCollisionCentral(const Eigen::VectorXd& state) const;
        bool inCollisionDecentral(const Eigen::VectorXd& state) const;
        bool checkWithPrior(const Eigen::Vector2d& state) const;
        bool checkK0condtion(const Eigen::Vector2d& state) const;
        bool avoidGoals(const Eigen::Vector2d& state) const;
        void addPath(const amp::Path& path);
        pair<VectorXd, VectorXd> getLimits();
        pair<int, Eigen::VectorXd> nearest;
        bool central;
    private:
        // amp::Problem2D problem;
        amp::MultiAgentProblem2D problem;
        vector<amp::Obstacle2D> obstacles;
        vector<amp::Path> computedPaths;
        vector<Regions> regions;
        vector<double> radii;
        pair<VectorXd, VectorXd> limits;
        double radius = 0;

};


class MyKinoChecker {
    public:
        MyKinoChecker(const amp::Problem2D& problem, vector<pair<double, double>> stateLimits, double w, double l) :
        stateLimits(stateLimits), w(w), l(l) {
            obstacles = ampToBoostObstacles(problem.obstacles);
        }
        bool inCollisionRectangle(const Eigen::VectorXd& state) const;
        vector<pair<double, double>> getLimits();

    private:
        vector<polygon> obstacles;
        vector<pair<double, double>> stateLimits;
        double w, l;
};