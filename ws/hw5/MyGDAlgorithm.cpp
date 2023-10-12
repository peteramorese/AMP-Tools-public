#include "MyGDAlgorithm.h"
#include "HelpfulClass.h"

using std::vector, std::string, Eigen::Vector2d, std::cout;

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    init(problem);
    while (!goalReached) {
        takeStep();
        if (step == 1000) break;
    }
    cout << "Goal reached in " << step << " steps\n";
    // path.waypoints.push_back(goal);
    return path;
}

void MyGDAlgorithm::init(const amp::Problem2D& problem) {
    int step = 1;
    start = problem.q_init;
    goal = problem.q_goal;
    obstacles = problem.obstacles;
    position = start;
    path.waypoints.push_back(start);
    stepSize = (goal - start).norm() / 100;
    goalReached = false;
    regions = findRegions(findEdges(problem))
}

void MyGDAlgorithm::takeStep() {
    position += getGradient() * stepSize;
    cout << "Position:\n" << position << "\n";
    path.waypoints.push_back(position);
    step++;
}

Vector2d MyGDAlgorithm::getGradient() {
    Vector2d gradient = - findAttractive();
	for (const vector<vector<Edges>>& polyRegions : regions) {
        break;
        gradient += findRepulsive(polyRegions);
    }
    cout << "gradient:\n" << gradient << "\n";
    if (gradient.norm() < E) goalReached = true;
    return gradient;
}

Vector2d MyGDAlgorithm::findAttractive(){
    double distanceToGoal = distanceBetweenPoints(position, goal);
    if (distanceToGoal <= dStar) {
        return zetta * (position - goal);
    } else {
        return dStar * zetta * (position - goal) / distanceToGoal;
    }
}

Vector2d MyGDAlgorithm::findRepulsive(const vector<vector<Edges>>& polyRegions) {
    <vector<Edges> region = findRegion(polyRegions);
    Vector2d closestPoint = findClosestPoint(region);
    Vector2d vectorFromObstacle = position - closestPoint;
    double dObstacle = vectorFromObstacle.norm()
    if (dObstacle <= qStar) {
        return eta * (1/qStar - 1/dObstacle) * (vectorFromObstacle/dObstacle)/pow(dObstacle, 2);
    } else {
        return 0;
    }
}

<vector<Edges> findRegion(const vector<vector<Edges>>& polyRegions) {
    bool left;
    bool allPass = false;
    for (const vector<Edge>& region : polyRegions) {
        if (region.size() == 2) left = false;
        else left = true;
        for (const Edge& edge : region) {
            if (!checkLine(position, edge, left)) break;
            allPass = true
        }
        if (allPass) return region;
    }
}

Vector2d findClosestPoint(const vector<Edges>& region) {
    Vector2d closestPoint;
    if (region.size() == 2) {
        closestPoint = region[0].points.second;
    } else {
        closestPoint = closestPointOnLine(position, edge);
    }
    return closestPoint;
}

vector<vector<vector<Edges>>> findRegions(const vector<vector<Edge>>& allEdges) {
    vector<vector<vector<Edges>>> regions;
    for (const vector<Edge>& polyEdges : allEdges) {
        bool first = true;
        vector<vector<Edges>> polyRegions;
        Edge line1, line3, firstLine;
        for (const Edge& edge : polyEdges) {
            line1 = findLineEquation(egde.points.first, pointA);
            if (!first) {
                polyRegions.push_back({line3, line1});
            } else {
                firstLine = line1;
                first = false;
            }
            line3 = findLineEquation(pointB, egde.points.second);
            polyRegions.push_back({line1, edge, line3});
        }
        polyRegions.push_back({line3, firstLine});
        regions.push_back(polyRegions);
    }
    return regions;
}