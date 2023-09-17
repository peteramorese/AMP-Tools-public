#include "Helper.h"
#include "MyBugAlgorithm.h"
#include <vector>
#include <cmath>
#include "tools/Obstacle.h" 
#include <list>
#include <Eigen/Geometry>

Helper::Helper(){};

Helper::~Helper(){};

bool Helper::isPointOnSegment(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const Eigen::Vector2d &point) {
    Eigen::Hyperplane<double, 2> line = Eigen::Hyperplane<double, 2>::Through(start, end);
    
    // Check if the point lies on the line
    if (std::abs(line.signedDistance(point)) > 1e-10) {
        return false;
    }
    
    // Check if the point lies within the bounds of the segment
    if ((point(0) >= std::min(start(0), end(0)) && point(0) <= std::max(start(0), end(0))) &&
        (point(1) >= std::min(start(1), end(1)) && point(1) <= std::max(start(1), end(1)))) {
        return true;
    }
    
    return false;
}

bool Helper::isIntersecting(const Eigen::Vector2d &vert1, const Eigen::Vector2d &vert2, const Eigen::Vector2d &vert3, const Eigen::Vector2d &vert4) {
    Eigen::Hyperplane<double,2> plane1 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);
    Eigen::Hyperplane<double,2> plane2 = Eigen::Hyperplane<double,2>::Through(vert3, vert4);
    Eigen::Vector2d intersection = plane1.intersection(plane2);
    if(isPointOnSegment(vert1, vert2, intersection)){
        return true;
    }
    return false;
}

Eigen::Vector2d Helper::getIntersect(const Eigen::Vector2d &vert1, const Eigen::Vector2d &vert2, const Eigen::Vector2d &vert3, const Eigen::Vector2d &vert4) {
    Eigen::Hyperplane<double,2> plane1 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);
    Eigen::Hyperplane<double,2> plane2 = Eigen::Hyperplane<double,2>::Through(vert3, vert4);
    Eigen::Vector2d intersection = plane1.intersection(plane2);
    return intersection;
}


std::list<Eigen::Vector2d> Helper::getObstacleTraverseVertices(std::vector<Eigen::Vector2d>& vertices, int j, Eigen::Vector2d position) {
    std::list<Eigen::Vector2d> traverseOrder;
    for(int i = j; i >= 0; i--){
        traverseOrder.push_back(vertices[i]);
    }
    for(int i = vertices.size() - 1; i > j; i--){
        traverseOrder.push_back(vertices[i]);
    }
    traverseOrder.push_back(position);
    return traverseOrder;
}

std::list<Eigen::Vector2d> Helper::shortestPath(Eigen::Vector2d position, Eigen::Vector2d goal, std::vector<Eigen::Vector2d> waypoints) {
    std::list<Eigen::Vector2d> path;
    auto it = std::find(waypoints.begin(), waypoints.end(), goal);
    int goalIndex = std::distance(waypoints.begin(), it);
    float leftDist = 0;
    float rightDist = 0;
    int positionRightIdx;
    int positionLeftIdx;
    for(int i = goalIndex; i >= 0; i--){
        leftDist += getDistance(waypoints[i], waypoints[i - 1]);
        if (waypoints[i] == position) {
            positionLeftIdx = i;
            break;
        }
    }
    for(int i = goalIndex; i < waypoints.size(); i++){
        rightDist += getDistance(waypoints[i], waypoints[i + 1]);
        if (waypoints[i] == position) {
            positionRightIdx = i;
            break;
        }
    }
    if(leftDist < rightDist){
        for(int i = positionLeftIdx + 1; i <= goalIndex; i++){
            path.push_back(waypoints[i]);
        }
    }
    else{
        for(int i = positionRightIdx - 1; i >= goalIndex; i--){
            path.push_back(waypoints[i]);
        }
    }
    return path;
}

float Helper::getDistance(Eigen::Vector2d point1, Eigen::Vector2d point2){
    return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
}

Eigen::Vector2d Helper::minDistance(Eigen::Vector2d point, std::vector<Eigen::Vector2d> waypoints){
    float minDist = MAXFLOAT;
    Eigen::Vector2d minDistPoint;
    for(int i = 0; i < waypoints.size() - 1; i++){
        float dist = getDistance(point, waypoints[i]);
        if(dist < minDist){
            minDist = dist;
            minDistPoint = waypoints[i];
        }
    }
    return minDistPoint;

}

std::vector<Eigen::Vector2d> Helper::expandObstacle(amp::Obstacle2D obstacle, float delta) const{
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

Eigen::Vector2d Helper::computeCentroid(const std::vector<Eigen::Vector2d>& points) const{
    Eigen::Vector2d centroid(0, 0);

    if (points.empty()) {
        return centroid;  // Return (0,0) if no points are provided.
    }

    for (const auto& point : points) {
        centroid += point;
    }

    return centroid / static_cast<double>(points.size());
}

Eigen::Vector2d Helper::expandVertex(Eigen::Vector2d vert1, Eigen::Vector2d vert2, Eigen::Vector2d vert3, Eigen::Vector2d vert4){
    Eigen::Hyperplane<double,2> plane1;
    Eigen::Hyperplane<double,2> plane2;
    Eigen::Vector2d intersection;
    plane1 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);
    plane2 = Eigen::Hyperplane<double,2>::Through(vert3, vert4);
    return plane1.intersection(plane2);
}

bool Helper::close(Eigen::Vector2d goal, Eigen::Vector2d position, float stepSize){
    Eigen::Vector2d direction1 = goal - position;
    if (direction1.norm() < stepSize) {
        return true;
    }
    return false;
}

Eigen::Vector2d Helper::shortestVectorDist(Eigen::Vector2d vert1, Eigen::Vector2d vert2, Eigen::Vector2d point){
    Eigen::Vector2d dir = vert2 - vert1;
    double lengthSquared = dir.squaredNorm();  // length of vert1-vert2 squared

    // If the segment is just a point, return the point
    if(lengthSquared == 0.0) return vert1;

    // Calculate the projection of point onto the line defined by vert1 and vert2
    double t = (point - vert1).dot(dir) / lengthSquared;

    // If t is in the [0, 1] range, it means the projection is within the segment
    if(t < 0.0) return vert1;
    else if(t > 1.0) return vert2;

    Eigen::Vector2d projection = vert1 + t * dir;
    return projection;
}



