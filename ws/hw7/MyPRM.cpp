#include "AMPCore.h"
#include "MyPRM.h"
#include <random>
#include <Eigen/Dense>
#include "HelpfulClass.h"
#include "MyAStar.h"

using namespace amp;
using std::vector, Eigen::VectorXd, Eigen::Vector2d, std::pair, std::size_t;

amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    Path2D path;
    limits.push_back({problem.x_min, problem.x_max});
    limits.push_back({problem.y_min, problem.y_max});
    VectorXd qRand;
    points[0] = problem.q_init;
    points[1] = problem.q_goal;
    int ind = 2;
    while (points.size() < n) {
        qRand = getRandomPoint();
        if (!isPointInCollision(qRand, problem.obstacles)) {
            // path.waypoints.push_back(qRand);
            points[ind] = qRand;
            ind++;
        }
    }
    connectNieghbors(problem.obstacles);
    ShortestPathProblem searchProblem = {graphPtr, 0, 1};
    // graphPtr->print();
    SearchHeuristic heuristic;
    MyAStarAlgo aStar(true);
    MyAStarAlgo::GraphSearchResult results = aStar.search(searchProblem, heuristic);
    for (const Node& node : results.node_path) {
        path.waypoints.push_back(points[node]);
    }
    return path;
}

VectorXd MyPRM::getRandomPoint() {
    int dim = 2;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::pair<double, double> limit;
    VectorXd randomPoint(dim);
    for (int i = 0; i < dim; i++) {
        limit = limits[i];
        std::uniform_real_distribution<double> dist(limit.first, limit.second);
        randomPoint(i) = dist(gen);
    }
    return randomPoint;
}

void MyPRM::connectNieghbors(const vector<amp::Obstacle2D> obstacles) { 
    double norm;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) { 
            norm = (points[i] - points[j]).norm();
            if (norm < r) {
                if (!isLineInCollision(points[i], points[j], obstacles)) {
                    graphPtr->connect(i, j, norm);
                    graphPtr->connect(j, i, norm);
                }
            }
        }
    }
}
