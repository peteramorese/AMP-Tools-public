#include "MyGDAlgorithm.h"
#include "HelpfulClass.h"

using std::vector, std::string, Eigen::Vector2d, std::cout;

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    int maxSteps = 2000;
    step = 0;
    init(problem);
    while (!goalReached) {
        takeStep();
        if (step > maxSteps) break;
    }
    cout << "Goal reached in " << step << " steps\n";
    path.waypoints.push_back(goal);
    return path;
}

void MyGDAlgorithm::init(const amp::Problem2D& problem) {
    start = problem.q_init;
    goal = problem.q_goal;
    obstacles = problem.obstacles;
    position = start;
    path.waypoints = {start};
    stepSize = (goal - start).norm() / 100;
    goalReached = false;
    regions = findRegions(findEdges(problem));
}

void MyGDAlgorithm::takeStep() {
    position += getGradient() * stepSize;
    // cout << "Position:\n" << position << "\n";
    path.waypoints.push_back(position);
    step++;
}

Vector2d MyGDAlgorithm::getGradient() {
    Vector2d gradient = - findAttractive();
	for (const vector<vector<Edge>>& polyRegions : regions) {
        gradient += findRepulsive(polyRegions);
    }
    double noise = 0.75 / gradient.norm();
    if (noise > 2) noise = 2;
    Vector2d randomGradient = {(double) rand() / (RAND_MAX) * noise - noise/2, (double) rand() / (RAND_MAX) * noise - noise/2};

    gradient += randomGradient;
    // cout << "gradient:\n" << gradient << "\n";
    // if (gradient.norm() < E) goalReached = true;
    return gradient;
}

Vector2d MyGDAlgorithm::findAttractive() {
    double distanceToGoal = distanceBetweenPoints(position, goal);
    if (distanceToGoal < E) goalReached = true;
    if (distanceToGoal <= dStar) {
        return zetta * (position - goal);
    } else {
        return dStar * zetta * (position - goal) / distanceToGoal;
    }
}

Vector2d MyGDAlgorithm::findRepulsive(const vector<vector<Edge>>& polyRegions) {
    vector<Edge> region = findRegion(polyRegions);
    Vector2d closestPoint = findClosestPoint(region);
    // cout << "\nClosest Point:\n" << closestPoint << "\n";
    Vector2d vectorFromObstacle = closestPoint - position;
    double dObstacle = vectorFromObstacle.norm();
    if (dObstacle < qStar) {
        Vector2d repulsiveGradient = eta * (1/qStar - 1/dObstacle) * (vectorFromObstacle/dObstacle)/pow(dObstacle, 2);
    	// cout << "Obstacle Gradient : (" << repulsiveGradient.x() << ", " << repulsiveGradient.y() << ")\n";
        return repulsiveGradient;
    } else {
        return {0, 0};
    }
}

vector<Edge> MyGDAlgorithm::findRegion(const vector<vector<Edge>>& polyRegions) {
    bool left;
    int ind = 0;
    for (const vector<Edge>& region : polyRegions) {
        bool allPass = true;
        if (region.size() == 2) left = false;
        else left = true;
        ind++;
        for (const Edge& edge : region) {
            if (!checkLine(position, edge, left)) {
                allPass = false; 
                break;
            }
        }
        if (allPass) return region;
    }
    cout << "\nCRASHED\n";
    goalReached = true;
    return polyRegions[0];
}

Vector2d MyGDAlgorithm::findClosestPoint(const vector<Edge>& region) {
    Vector2d closestPoint;
    if (region.size() == 2) closestPoint = region[0].points.first;
    else closestPoint = closestPointOnLine(position, region[1]);
    return closestPoint;
}

