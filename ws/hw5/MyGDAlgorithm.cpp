#include "MyGDAlgorithm.h"
#include "HelpfulClass.h"

using std::vector, std::string, Eigen::Vector2d, std::cout;

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    init(problem);
    while (!goalReached) {
        takeStep();
        cout << "step\n";
    }
    path.waypoints.push_back(goal);
    return path;
}

void MyGDAlgorithm::init(const amp::Problem2D& problem) {
    start = problem.q_init;
    goal = problem.q_goal;
    obstacles = problem.obstacles;
    position = start;
    path.waypoints.push_back(start);
    stepSize = (goal - start).norm() / 500;
    goalReached = false;
    E = stepSize;
    dStar = E;
    zetta = E;
}

void MyGDAlgorithm::takeStep() {
    position += getGradient() * stepSize;
    path.waypoints.push_back(position);
}

Vector2d MyGDAlgorithm::getGradient() {
    Vector2d gradient = findAttractive();
	for (const amp::Obstacle2D& obstacle : obstacles) {
        gradient += findRepulsive(obstacle);
    }
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

Vector2d MyGDAlgorithm::findRepulsive(const amp::Obstacle2D& obstacle) {
    double distanceToGoal = distanceBetweenPoints(position, goal);
    if (distanceToGoal <= dStar) {
        return zetta * (position - goal);
    } else {
        return dStar * zetta * (position - goal) / distanceToGoal;
    }
}

double distanceToObstacle() {
    return 1.0;
}