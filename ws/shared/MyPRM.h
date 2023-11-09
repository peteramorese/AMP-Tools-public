#include "AMPCore.h"
#include "hw/HW7.h"
#include "CSpaceConstructor.h"

using namespace amp;
using std::vector, Eigen::Vector2d, std::pair, std::size_t;

struct MyNode {
    int ind;
    Eigen::VectorXd point;
    double cost;
    int parent;
};

struct MyPoint {
    int ind;
    Eigen::Vector2d point;
};

class MyPRM : public amp::PRM2D {
    public:
        MyPRM(int n, double r, bool smooth)
        : n(n), r(r), smooth(smooth) {graphPtr = std::make_shared<amp::Graph<double>>();}
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        Eigen::VectorXd getRandomPoint();
        void connectNieghbors(const vector<amp::Obstacle2D> obstacles);
        std::shared_ptr<amp::Graph<double>> getGraph();
        std::map<uint32_t, Vector2d> getPoints();
    private:
        int n;
        double r;
        bool smooth;
        std::map<uint32_t, Vector2d> points;
        vector<std::pair<double, double>> limits;
        std::shared_ptr<amp::Graph<double>> graphPtr;
        // Eigen::VectorXd init, goal;
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        MyRRT(int n, double r, bool smooth)
        : n(n), r(r), smooth(smooth) { graphPtr = std::make_shared<amp::Graph<double>>();}
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        Eigen::VectorXd getRandomPoint();
        void connectNieghbors(const vector<amp::Obstacle2D> obstacles);
        pair<int, Eigen::VectorXd> findNearest(const Eigen::VectorXd& point, const vector<amp::Obstacle2D> obstacles);
        std::shared_ptr<amp::Graph<double>> getGraph();
        std::map<uint32_t, Vector2d> getPoints();
    private:
        int n;
        double r;
        bool smooth;
        std::map<uint32_t, Vector2d> points;
        std::map<uint32_t, uint32_t> parents;
        vector<std::pair<double, double>> limits;
        std::shared_ptr<amp::Graph<double>> graphPtr;
};
