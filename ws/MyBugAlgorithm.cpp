#include "MyBugAlgorithm.h"
#include <vector>
#include <cmath>
#include "tools/Obstacle.h" 
#include "Line.h"
#include <list>

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) const {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    Eigen::Vector2d step; // robot's next potential position
    Eigen::Vector2d robotPosition(problem.q_init[0] , problem.q_init[1]); // robot's current position
    Eigen::Vector2d goalPosition(problem.q_goal[0] , problem.q_goal[1]); // goal position
    amp::Obstacle2D hitObstacle;
    Eigen::Vector2d vert1 = robotPosition;
    Eigen:: Vector2d vert2 = goalPosition;
    std::list<std::pair<Eigen::Vector2d, Eigen::Vector2d>> traverseList; // list of vertices on obstacle.
    std::list<Eigen::Vector2d> safeTraverseList; // list of vertices to traverse around obstacle.
    float delta = 0.01; // step Size
    float obstaclePadding = 0.1; // distance to stay away from obstacle
    float near = 0.1; // distance to vertex to be considered at vertex.
    float closestDistance = distance(robotPosition, goalPosition);
    float currentDistance = distance(robotPosition, goalPosition);
    Eigen::Vector2d closestVert = robotPosition;
    Eigen::Vector2d hitVert; // position just before bug hits obstacle.
    std::vector<Eigen::Vector2d> rightTraverseWaypoints;
    std::vector<Eigen::Vector2d> leftTraverseWaypoints;
    path.waypoints.push_back(robotPosition);
    int run = 0;

    while (run < 2){
        // head towards goal:
        step = robotPosition;
        step = stepLine2(step, vert2, delta);
        while ( !occupied(step, problem, traverseList, near) && distance(step, problem.q_goal) > near){
            robotPosition = step;
            step = stepLine2(step, vert2, delta);
        }
        if (distance(robotPosition, problem.q_goal) < near){
            robotPosition = problem.q_goal;
            return path;
        }

        path.waypoints.push_back(robotPosition);
        hitVert = robotPosition;
        closestDistance = distance(robotPosition, goalPosition);
        safeTraverseList = traverseObstacle(robotPosition, traverseList, obstaclePadding);
        
        // traverse around obstacle:
        for (Eigen::Vector2d vertex : safeTraverseList){
            step = stepLine2(robotPosition, vertex, delta);
            while ( !occupied(step, problem, traverseList, near) && distance(step, vertex) > near){
                robotPosition = step;
                path.waypoints.push_back(robotPosition);
                currentDistance = distance(robotPosition, goalPosition);
                if (currentDistance < closestDistance){
                    closestDistance = currentDistance;
                    closestVert = robotPosition;
                }
                if (distance(step, problem.q_goal) < near){
                    robotPosition = step;
                    path.waypoints.push_back(problem.q_goal);
                    return path;
                }
                step = stepLine2(robotPosition, vertex, delta);
            }
            robotPosition = vertex;
            currentDistance = distance(robotPosition, goalPosition);
            if (currentDistance < closestDistance){
                    closestDistance = currentDistance;
                    closestVert = robotPosition;
            }
            path.waypoints.push_back(robotPosition);
        }
        path.waypoints.push_back(hitVert);

        // traverse towards closest distance to goal on obstacle:
        for (Eigen::Vector2d point : returnToMinDist(hitVert, closestVert, path)){
            robotPosition = point;
            path.waypoints.push_back(point);
        }
        run++;
    }

    return path;


}

/**
 * @brief Describe the purpose of the function here
 * 
 * @param paramName Description of parameter
 * @return returnType Description of return value
 **/
std::vector<Eigen::Vector2d> MyBugAlgorithm::enlargeObstacle(amp::Obstacle2D obstacle, float delta) const{
    std::vector<Eigen::Vector2d> enlargedObstacle;
    std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
    Eigen::Vector2d centroid = computeCentroid(vertices);
    for (Eigen::Vector2d vertex : vertices){
        Eigen::Vector2d enlargedVertex;
        if (vertex[0] < centroid[0]){
            enlargedVertex[0] = vertex[0] - delta;
        }
        else{
            enlargedVertex[0] = vertex[0] + delta;
        }
        if (vertex[1] < centroid[1]){
            enlargedVertex[1] = vertex[1] - delta;
        }
        else{
            enlargedVertex[1] = vertex[1] + delta;
        }
        enlargedObstacle.push_back(enlargedVertex);
    }
    return enlargedObstacle;
}

/**
 * @brief Describe the purpose of the function here
 * 
 * @param paramName Description of parameter
 * @return returnType Description of return value
 **/
Eigen::Vector2d MyBugAlgorithm::computeCentroid(const std::vector<Eigen::Vector2d>& points) const{
    Eigen::Vector2d centroid(0, 0);

    if (points.empty()) {
        return centroid;  // Return (0,0) if no points are provided.
    }

    for (const auto& point : points) {
        centroid += point;
    }

    return centroid / static_cast<double>(points.size());
}

/**
 * @brief Given a goal vert and a robot vert that appears twice on the path, return the points
 * to return to the goal vert in minimum distance.
 * 
 * @param paramName Description of parameter
 * @return returnType Description of return value
 **/
std::vector<Eigen::Vector2d> MyBugAlgorithm::returnToMinDist(Eigen::Vector2d vert1, Eigen::Vector2d vert2, amp::Path2D path) const{
    float distLeft;
    float distRight;
    std::vector<Eigen::Vector2d> waypointLeft;
    std::vector<Eigen::Vector2d> waypointRight;
    std::vector<Eigen::Vector2d>::iterator itLeft = std::find(path.waypoints.begin(), path.waypoints.end(), vert1);
    std::vector<Eigen::Vector2d>::iterator itRight = std::find(path.waypoints.begin(), path.waypoints.end(), vert2);
    // check length of left traversal.
    for (auto traverse = itLeft; traverse != path.waypoints.end(); ++traverse) {
        waypointLeft.push_back(*traverse);
        if(*traverse == vert2)
            break;
        distLeft += distance(*traverse, *(traverse + 1));
    }
    // check length of right traversal.
    for (auto traverse = itRight; traverse != path.waypoints.end(); ++traverse) {
        waypointRight.push_back(*traverse);
        if(*traverse == vert1)
            break;
        distRight += distance(*traverse, *(traverse + 1));
    }
    if (distLeft < distRight){
        return waypointLeft;
    }
    else{
        std::reverse(waypointRight.begin(), waypointRight.end());
        return waypointRight;
    }
}

/**
 * @brief Returns distance between two vertices.
 * 
 * @param vert1 Fromt point
 * @param vert2 To point
 * @return returnType Description of return value
 **/
float MyBugAlgorithm::distance(Eigen::Vector2d vert1, Eigen::Vector2d vert2) const{
    return std::sqrt(std::pow(vert2[0] - vert1[0], 2) + std::pow(vert2[1] - vert1[1], 2));
}

/**
 * @brief Takes a step in direct direction towards vert2.
 * 
 * @param vert1 From point
 * @param vert2 To point
 * @return returnType Description of return value
 **/
Eigen::Vector2d MyBugAlgorithm::stepLine2(Eigen::Vector2d vert1, Eigen::Vector2d vert2, float delta) const{
    // Handle vertical line case where m = inf:
    if (abs(vert1[0] - vert2[0]) < delta) {
       if (vert2[1] > vert1[1]){
            return Eigen::Vector2d(vert1[0], vert1[1] + delta);
       }
       else{
            return Eigen::Vector2d(vert1[0], vert1[1] - delta);
       }
    }
    float m = (vert2[1] - vert1[1]) / (vert2[0] - vert1[0]);
    float stepX = ( vert2[0] > vert1[0] ) ? vert1[0] + delta : vert1[0] - delta;
    float stepY = m * (stepX - vert1[0]) + vert1[1];
    return Eigen::Vector2d (stepX, stepY);
}

void MyBugAlgorithm::stepLine(Eigen::Vector2d vert1, Eigen::Vector2d vert2, Eigen::Vector2d& step, float delta) const {
    
    // Handle vertical line case:
    if (vert1[0] == vert2[0]) {
        if (vert1[0] > step[1]) {
            step = Eigen::Vector2d (step[0], step[0] - delta);
        } else {
            step = Eigen::Vector2d (step[0], step[0] + delta);
        }
    }

    float m = (vert2[1] - vert1[1]) / (vert2[0] - vert1[0]);
    float stepX = ( vert1[0] > vert2[0] ) ? step[0] - delta : step[0] + delta;
    float stepY = m * (stepX - step[0]) + step[1];
    step = Eigen::Vector2d (stepX, stepY);
}

Eigen::Vector2d MyBugAlgorithm::stepToGoal(const amp::Problem2D& problem, Eigen::Vector2d location, float delta) const {
    // Handle vertical line case:
    if (location[0] == problem.q_goal[0]) {
        if (location[0] > problem.q_goal[1]) {
            return Eigen::Vector2d (location[0], location[0] - delta);
        } else {
            return Eigen::Vector2d (location[0], location[0] + delta);
        }
    }

    float m = (problem.q_goal[1] - location[1]) / (problem.q_goal[0] - location[0]);
    float stepX = ( problem.q_goal[0] > location[0] ) ? location[0] + delta : location[0] - delta;
    float stepY = m * (stepX - location[0]) + location[1];
    return Eigen::Vector2d (stepX, stepY);
}

/**
 * @brief Creates a slightly larger set of vertices around obstacle for the bug to safely approach.
 * 
 * @param location current bug position
 * @param reference to list of vertices of obstacle in CW order ending with bug intersect position.
 * @return returnType Description of return value
 **/
std::list<Eigen::Vector2d> MyBugAlgorithm::traverseObstacle(Eigen::Vector2d location, std::list<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& traverseList, float delta) const{
    std::list<Eigen::Vector2d> safeTraverseList;
    for (std::pair<Eigen::Vector2d, Eigen::Vector2d> line : traverseList){
        Eigen::Vector2d vert1 = line.first;
        Eigen::Vector2d vert2 = line.second;
        Eigen::Vector2d vertSafe;

        if (vert1[1] == vert2[1] and vert2[0] > vert1[0]) {
            vertSafe = Eigen::Vector2d (vert2[0] + delta, vert2[1] + delta);
        }
        else if (vert1[1] == vert2[1] and vert2[0] < vert1[0]) {
            vertSafe = Eigen::Vector2d (vert2[0], vert2[1] - delta);
        }
        else if (vert1[0] == vert2[0] and vert2[1] > vert1[1]) {
            vertSafe = Eigen::Vector2d (vert2[0] - delta, vert2[1] + delta);
        }
        else if (vert1[0] == vert2[0] and vert2[1] < vert1[1]){
            vertSafe = Eigen::Vector2d (vert2[0] + delta, vert2[1] - delta);
        }
        else if (vert2[0] < vert1[0] && vert2[1] < vert1[1]){ // / line
            vertSafe = Eigen::Vector2d (vert2[0], vert2[1] - delta);
        }
        else if (vert2[1] > vert1[1] && vert2[0] < vert1[0]){ // \ line
            vertSafe = Eigen::Vector2d (vert2[0] - delta, vert2[1] + delta);
        }
        else if (vert2[1] > vert1[1] && vert2[0] > vert1[1]){
            vertSafe = Eigen::Vector2d (vert2[0] - delta, vert2[1] - delta);
        }
        else {
            vertSafe = Eigen::Vector2d (vert2[0] + delta, vert2[1]);
        }
        safeTraverseList.push_back(vertSafe);
    }
    return safeTraverseList;
}

bool MyBugAlgorithm::occupied(Eigen::Vector2d location, const amp::Problem2D& problem, std::list<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& traverseList, float near) const {
    // given a robot location (x,y) check to see if the location is occupied by an obstacle.
    bool isOccupied = false;
    for (amp::Obstacle2D obstacle : problem.obstacles){
        std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
        for (size_t i = 0; i < vertices.size(); ++i){
            if (i == vertices.size() - 1) {
                if (isPointOnLine(vertices[i], vertices[0], location, near)) {
                    traverseList = traverseOrder(vertices, i, location);
                    return true;
                }
            }
            else{
                if (isPointOnLine(vertices[i], vertices[i+1], location, near)) {
                    traverseList = traverseOrder(vertices, i+1, location);
                    return true;
                }
            }
        }
    }
    return false;
}

/**
 * @brief creates a list of vertices around an obstacle in a CW order (bug must turn left). Order ends with returning back to bugs current position.
 * 
 * @param location Description of parameter
 * @return returnType Description of return value
 **/
std::list<std::pair<Eigen::Vector2d, Eigen::Vector2d>> MyBugAlgorithm::traverseOrder(std::vector<Eigen::Vector2d>& vertices, int destination1, Eigen::Vector2d location) const{

    int n = vertices.size();
    std::list<std::pair<Eigen::Vector2d, Eigen::Vector2d>> vertexList;
    for (size_t i = 0; i < vertices.size(); i++){
        Eigen::Vector2d curr = vertices[(destination1 - i + n) % n];
        Eigen::Vector2d next = vertices[(destination1 - i - 1 + n) % n];
        if (i == 0){
            curr = location;
        }
        vertexList.push_back(std::make_pair(curr, next));
        if (i == vertices.size() - 1){
            // if on obstacle alst vertex add line to bug intersect position.
            vertexList.push_back(std::make_pair(next, location));
        }
    }
    return vertexList;
}

bool MyBugAlgorithm::isPointOnLine(Eigen::Vector2d vert1, Eigen::Vector2d vert2, Eigen::Vector2d location, float near) const {
    // Given two vertices, check to see if the robot location is on the line between them.
    if (vert1[0] == vert2[0]) {
        return std::abs(location[0] - vert1[0]) < near;
    }
    float m = (vert2[1] - vert1[1]) / (vert2[0] - vert1[0]);
    return std::abs(location[1] - vert1[1] - m * (location[1] - vert1[0])) < near;
}
