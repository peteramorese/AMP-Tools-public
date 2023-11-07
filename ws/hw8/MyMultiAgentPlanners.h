#include "AMPCore.h"
#include "hw/HW8.h"
#include "HelpfulClass.h"

using namespace amp;
using std::vector, Eigen::VectorXd, std::pair, std::size_t;
using Regions = vector<vector<double>>;

class MyCentralPlanner : public CentralizedMultiAgentRRT {
    public:
        MyCentralPlanner(int n, double r, double p)
        : n(n), r(r), p(p) {}
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override; 
    private:
        int n, m;
        double r, p;
        VectorXd init, goal;
        vector<double> radii;
        vector<Regions> allRegions;

};