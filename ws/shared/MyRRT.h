#include "AMPCore.h"
#include "hw/HW7.h"
#include "HelpfulClass.h"
#include "MyCollisionCheckers.h"

using namespace amp;
using std::vector, Eigen::VectorXd, std::pair;

class MyGenericRRT {
    public:
        MyGenericRRT(int n, double r, double p, double eps, vector<std::pair<double, double>> limits)
        : n(n), r(r), p(p), eps(eps), limits(limits) {}
        amp::Path plan(const VectorXd& init_state, const VectorXd& goal_state, MyCentralChecker& collision_checker); 
        Eigen::VectorXd getRandomPoint(const pair<VectorXd, VectorXd>& limits);
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