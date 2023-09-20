#include "MyBugAlgorithm.h"
#include <vector>
#include <cmath>
#include "tools/Obstacle.h" 
#include "Bug1.h"
#include <list>
#include <Eigen/Geometry>
#include "Helper.h"

Bug1::Bug1(const amp::Problem2D& p, float sZ, float d): stepSize(sZ), delta(d){
    position = p.q_init;
    goalQueue.push_back(p.q_goal);
    environment = p;
    mode = 0;
};

Bug1::~Bug1(){};

void Bug1::step(){

    Helper helper = Helper();

    // step 1: Take potential step to first goal point in queue.
    Eigen::Vector2d newPosition = goalQueue.front();

    if (mode == 1 && helper.isPointOnSegment(position, newPosition, hitPoint) && position != hitPoint && helper.pathIsClear(position, hitPoint, environment)){
        goalQueue.clear();
        position = hitPoint;
        waypoints.push_back(position);
        leavePoint = helper.minDistance(environment.q_goal, waypoints);
        goalQueue = helper.shortestPath(position, leavePoint, waypoints);
        mode = 2;
        return;
    }

    if (mode == 2 && newPosition == leavePoint){
        goalQueue.clear();
        position = newPosition;
        waypoints.push_back(position);
        goalQueue.push_back(environment.q_goal);
        mode = 0;
        return;
    }
    

    //step 2: Check to see if new position crossed any any obstacles. If yes, make position delta before first intersection and update goalQueue.
    Eigen::Hyperplane<double,2> plane1;
    Eigen::Hyperplane<double,2> plane2;
    Eigen::Vector2d vert1;
    Eigen::Vector2d vert2;
    Eigen::Vector2d intersection;
    Eigen::Vector2d minIntersectionVert;
    Eigen::Vector2d minSafeVert;
    std::vector<Eigen::Vector2d> expandedVertices;
    float minIntersectionDist = MAXFLOAT;
    float currIntersectionDist;
    bool hit = false;

    plane1 = Eigen::Hyperplane<double,2>::Through(position, newPosition);
    for(amp::Obstacle2D obstacle : environment.obstacles){
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
        for(size_t i = 0; i < vertices.size(); i++){
            if (i == vertices.size() - 1) {
                // create line from two vertices.
                vert1 = vertices[0];
                vert2 = vertices[i];
                plane2 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);
                intersection = plane1.intersection(plane2);

                //check to see if intersection is on both line segments.
                // case 1: new position is in obstacle.
                if(helper.isPointOnSegment(position, newPosition, intersection) && helper.isPointOnSegment(vert1, vert2, intersection) && helper.getDistance(intersection, position) < minIntersectionDist){
                    minIntersectionDist = helper.getDistance(intersection, position);
                    expandedVertices = helper.expandObstacle(obstacle, delta);
                    minSafeVert = helper.expandVertex(expandedVertices[0], expandedVertices[i], position, newPosition);
                    goalQueue = helper.getObstacleTraverseVertices(expandedVertices, i, minSafeVert);
                    hit = true;
                }
                
            }
            else{
                // create line from two vertices.
                vert1 = vertices[i];
                vert2 = vertices[i+1];
                plane2 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);
                intersection = plane1.intersection(plane2);
                
                //check to see if intersection is on both line segments.
                // case 1: new position is in obstacle.
                if(helper.isPointOnSegment(position, newPosition, intersection) && helper.isPointOnSegment(vert1, vert2, intersection) && helper.getDistance(intersection, position) < minIntersectionDist){
                    minIntersectionDist = helper.getDistance(intersection, position);
                    expandedVertices = helper.expandObstacle(obstacle, delta);
                    minSafeVert = helper.expandVertex(expandedVertices[i], expandedVertices[i+1], position, newPosition);
                    goalQueue = helper.getObstacleTraverseVertices(expandedVertices, i, minSafeVert);
                    hit = true;
                }
            }
        }
    }
    if(hit){
        Eigen::Vector2d cloestVert = helper.shortestVectorDist(position, minSafeVert, environment.q_goal);
        waypoints.push_back(minSafeVert);
        position = minSafeVert;
        if(position == goalQueue.front()){
            goalQueue.pop_front();
        }
        if(mode == 1){
            goalQueue.push_back(hitPoint);
        }
        if(mode == 0){
            // Tansitioning to boundry follow mode. Make bug return to this point.
            mode = 1;
            hitPoint = position;
        }
        return;
    }
    Eigen::Vector2d cloestVert = helper.shortestVectorDist(position, newPosition, environment.q_goal);
    waypoints.push_back(cloestVert);
    position = newPosition;
    goalQueue.pop_front();
};