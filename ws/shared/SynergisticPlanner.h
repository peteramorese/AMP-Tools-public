#include "AMPCore.h"
#include "KinoRRT.h"
#include "Triangulate.h"
#include "DFA.h"
using std::vector, Eigen::VectorXd, Eigen::Vector2d, std::pair;

class SynergisticPlanner : public ProductAutomaton, public KinoRRT {
public:
    // Constructor inheriting from ProductAutomaton constructor
    SynergisticPlanner(const DFA& dfa, const std::unordered_map<uint32_t, abstractionNode>& abstraction, int n, double r, double p, vector<pair<double, double>> controlLimits)
        : ProductAutomaton(dfa, abstraction), KinoRRT(n, r, p, controlLimits) {
        hybridMap = getHybridMap();
        hybridMapBack = getHybridMapBack();
        dynamicWeights = getWeightMap();
        neighborMap = getNeighborMap();
    }

    amp::Path synergisticPlan(const VectorXd& init_state, MyKinoChecker& collision_checker) {
        amp::Path path;
        amp::Path subPath;
        const std::unordered_map<uint32_t, abstractionNode>& M = getAbstraction();
        for (const pair<uint32_t, abstractionNode>& faces : M) {
            std::vector<Eigen::Vector2d> vertices(faces.second.vertices.begin(), faces.second.vertices.end());
            if (isPointInsidePolygon({init_state(0), init_state(1)}, vertices)) {
                d_init = faces.first;
            }
        }
        uint32_t HS_init = hybridMapBack[{0, d_init}];
        int iter = 0;
        while (iter < 200)
        {
            path.waypoints = {};
            std::cout << "Starting at region: " << d_init << std::endl;
            std::cout << "Starting at HS: " << HS_init << std::endl;
            std::list<uint32_t> highLevelPlan = searchPath(HS_init, dynamicWeights).node_path;
            std::cout << "High level path: ";
            for (uint32_t node : highLevelPlan) std::cout << node << " ";
            std::cout << "\nHigh level region: ";
            for (uint32_t node : highLevelPlan) std::cout << hybridMap[node].second << " ";
            std::cout << std::endl;
            auto last = std::prev(highLevelPlan.end());
            VectorXd xStart;
            bool first = true;
            uint32_t prevHS = HS_init;
            for (auto it = highLevelPlan.begin(); it != last; ++it) {
                uint32_t HS = *it;
                if (first) {
                    xStart = init_state; 
                    first = false;
                }
                uint32_t di = hybridMap[HS].second;
                std::cout << "\nGOING TO STATE " << HS << " with region " << di << " and mode " << hybridMap[HS].first << std::endl;
                std::vector<Eigen::Vector2d> nextRegion(M.at(di).vertices.begin(), M.at(di).vertices.end());
                for (const auto& vec : nextRegion) std::cout << "(" << vec.x() << ", " << vec.y() << ")" << std::endl;
                
                uint32_t i = 0;
                for (;i < neighborMap[prevHS].size(); ++i)
                    if (neighborMap[prevHS][i] == HS) break;

                subPath = plan(xStart, nextRegion, collision_checker, dynamicWeights[prevHS][i]);
                cout << "New Weight: " << dynamicWeights[prevHS][i] << std::endl;
                clearRRT();
                if (!subPath.valid) break;
                xStart = subPath.waypoints.back();
                xStart.conservativeResize(init_state.size());
                path.waypoints.insert(path.waypoints.end(), subPath.waypoints.begin(), subPath.waypoints.end());
                prevHS = HS;
            }
            if (subPath.valid) return path;
            iter++;
        }
        return path;
    
    }

private:
    uint32_t d_init; 
    std::map<uint32_t, std::pair<uint32_t, uint32_t>> hybridMap;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> hybridMapBack;
    std::map<uint32_t, std::vector<double>> dynamicWeights;
    std::map<uint32_t, std::vector<uint32_t>> neighborMap;
};