#include "AMPCore.h"
#include "KinoRRT.h"
#include "Triangulate.h"
#include "DFA.h"
using std::vector, Eigen::VectorXd, Eigen::Vector2d, std::pair;

class SynergisticPlanner : public ProductAutomaton, public KinoRRT {
public:
    // Constructor inheriting from ProductAutomaton constructor
    SynergisticPlanner(const DFA& dfa, const std::unordered_map<uint32_t, xState>& abstraction, int n, double r, double p, vector<pair<double, double>> controlLimits)
        : ProductAutomaton(dfa, abstraction), KinoRRT(n, r, p, controlLimits) {
        hybridMap = getHybridMap();
    }

    amp::Path synergisticPlan(const VectorXd& init_state, MyKinoChecker& collision_checker) {
        amp::Path path;
        const std::unordered_map<uint32_t, xState>& M = getAbstraction();
        for (const pair<uint32_t, xState>& faces : M) {
            std::vector<Eigen::Vector2d> vertices(faces.second.vertices.begin(), faces.second.vertices.end());
            if (isPointInsidePolygon({init_state(0), init_state(1)}, vertices)) {
                d_init = faces.first;
            }
        }
        int iter = 0;
        while (iter < 10)
        {
            std::list<uint32_t> highLevelPlan = searchPath(d_init).node_path;
            std::cout << "High level path: ";
            for (uint32_t node : highLevelPlan) std::cout << node << " ";
            std::cout << std::endl;
            VectorXd xStart;
            bool first = true;
            for (uint32_t pi : highLevelPlan){
                if (first) xStart = init_state; 
                uint32_t di = hybridMap[pi].second;
                std::cout << "GOING TO REGION: " << di << std::endl;
                std::vector<Eigen::Vector2d> nextRegion(M.at(di).vertices.begin(), M.at(di).vertices.end());
                amp::Path subPath = plan(xStart, nextRegion, collision_checker);
                clearRRT();
                if (!subPath.valid) break;
                xStart = subPath.waypoints.back();
            }
            if (path.valid) return path;
            iter++;
        }
        return path;
    
    }

private:
    uint32_t d_init; 
    std::map<uint32_t, std::pair<uint32_t, uint32_t>> hybridMap;
    std::map<std::pair<uint32_t, uint32_t>, double> dynamicWeights;
};