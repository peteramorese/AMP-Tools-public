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

    if (results.success) {
        for (const Node& node : results.node_path) {
            path.waypoints.push_back(points[node]);
        }
        if (smooth) smoothPath(path, problem.obstacles);
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

std::shared_ptr<amp::Graph<double>> MyPRM::getGraph() {
    return graphPtr;
}

std::map<uint32_t, Vector2d> MyPRM::getPoints() {
    return points;
};

amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    Path2D path;
    limits.push_back({problem.x_min, problem.x_max});
    limits.push_back({problem.y_min, problem.y_max});
    VectorXd qRand, nearest;
    points[0] = problem.q_init;
    std::map<int, int> parents;
    int ind = 1;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0, 1);    
    bool success = false;
    while (points.size() < n) {
        double goalBias = dist(gen);
        if (goalBias > 0.95) {
            cout << "Goal Bias\n";
            qRand = problem.q_goal;
        } else {
            qRand = getRandomPoint();
        }
        pair<int, VectorXd> nearest = findNearest(qRand, problem.obstacles);
        if (nearest.first != -1) {
            graphPtr->connect(nearest.first, ind, (points[nearest.first] - nearest.second).norm());
            points[ind] = nearest.second;
            parents[ind] = nearest.first;
            // cout << "Adding point: ( " << nearest.second(0) << ", " << nearest.second(1) << ")\n"; 
            ind++;
            if ((nearest.second - problem.q_goal).norm() < 0.1) {
                cout << "Goal found\n";
                success = true;
                break;
            }
        }
    }
    ind--;
    int node = ind;
    while (node != 0) {
        // cout << "Adding node " << node << " at ( " << points[node](0) << ", " << points[node](1) << ")\n"; 
        path.waypoints.insert(path.waypoints.begin(), points[node]);
        node = parents[node];
    }
    path.waypoints.insert(path.waypoints.begin(), problem.q_init);
    if (success) path.waypoints.push_back(problem.q_goal);
    return path;
}

pair<int, VectorXd> MyRRT::findNearest(const VectorXd& point, const vector<amp::Obstacle2D> obstacles) {
    VectorXd nearest = points[0];
    int ind = 0;
    for (int i = 1; i < points.size(); i++) {
        if ((point - points[i]).norm() < (point - nearest).norm()) {
            nearest = points[i];
            ind = i;
        }
    }
    int steps = 10;
    VectorXd step = (point - nearest)/steps;
    VectorXd endpoint = nearest;
    for (int i = 0; i < steps; i++) {
        endpoint += step;
        if (isLineInCollision(nearest, endpoint, obstacles)) { 
            endpoint -= step;
            if (i == 0) ind = -1;
            break;
        }
    }
    return {ind, endpoint};
}

VectorXd MyRRT::getRandomPoint() {
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
