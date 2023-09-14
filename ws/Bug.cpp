#include "MyBugAlgorithm.h"
#include <vector>
#include <cmath>
#include "tools/Obstacle.h" 
#include "Bug.h"
#include <list>
#include <Eigen/Geometry>
#include "Helper.h"

Bug::Bug(const amp::Problem2D& p, float sZ): stepSize(sZ){
    position = p.q_init;
    goalQueue.push_back(p.q_goal);
    environment = p;
    mode = 0;
};

Bug::~Bug(){};

void Bug::step(){

    Helper helper = Helper();

    if(mode == 0 | mode == 1){
        Eigen::Vector2d direction1 = goalQueue.front() - position;

        //1 . check if goal is reached.
        if (direction1.norm() < stepSize) {
            position = goalQueue.front();
            goalQueue.pop_front(); 
            return;
        }

        // 2. Calculate potential step.
        direction1.normalize();
        Eigen::ParametrizedLine<double, 2> line1(position, direction1);
        Eigen::Vector2d newPosition = line1.pointAt(stepSize);

        // 3. Check to see if new position is in obstacle.
        Eigen::Hyperplane<double,2> plane1;
        Eigen::Hyperplane<double,2> plane2;
        Eigen::Vector2d vert1;
        Eigen::Vector2d vert2;
        Eigen::Vector2d intersection;
        plane1 = Eigen::Hyperplane<double,2>::Through(position, newPosition);
        for(amp::Obstacle2D obstacle : environment.obstacles){
            std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
            for(size_t i = 0; i < vertices.size(); i++){
                if (i == vertices.size() - 1) {
                    // create line from two vertices.
                    vert1 = vertices[i];
                    vert2 = vertices[0];
                    plane2 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);

                    // find intersection of bug line and obstacle line.
                    intersection = plane1.intersection(plane2);
                    if(intersection[0] == position[0] && intersection[1] == position[1]){
                        intersection[0] = MAXFLOAT;
                        intersection[1] = MAXFLOAT;
                    }

                    //check to see if intersection is on both line segments.
                    // case 1: new position is in obstacle.
                            // make bug position on obstacle.
                    if(helper.isPointOnSegment(position, newPosition, intersection) && helper.isPointOnSegment(vert1, vert2, intersection)){
                        position = intersection;
                        // case 1: transition from direct path mode to boundry follow mode.
                        if (mode == 0) {
                            hitPoint = position;
                            minDist = (position - environment.q_goal).norm();
                            minDistPoint = position;
                        }
                        mode = 1; // mode is boundry following.
                        goalQueue = helper.getObstacleTraverseVertices(vertices, 0, position);
                        return;
                    }
                    
                }
                else{
                    // create line from two vertices.
                    vert1 = vertices[i];
                    vert2 = vertices[i+1];
                    plane2 = Eigen::Hyperplane<double,2>::Through(vert1, vert2);

                    // find intersection of bug line and obstacle line.
                    intersection = plane1.intersection(plane2);
                    if(intersection[0] == position[0] && intersection[1] == position[1]){
                        intersection[0] = MAXFLOAT;
                        intersection[1] = MAXFLOAT;
                    }
                    
                    //check to see if intersection is on both line segments.
                    // case 1: new position is in obstacle.
                            // make bug position on obstacle.
                    if(helper.isPointOnSegment(position, newPosition, intersection) && helper.isPointOnSegment(vert1, vert2, intersection)){
                        position = intersection;
                        // case 1: transition from direct path mode to boundry follow mode.
                        if (mode == 0) {
                            hitPoint = position;
                            minDist = (position - environment.q_goal).norm();
                            minDistPoint = position;
                        }
                        mode = 1; // mode is boundry following.
                        goalQueue = helper.getObstacleTraverseVertices(vertices, i, position);
                        return;
                    }
                }
            }
        }
        // case 2: new position is not in obstacle.
            // make bug position new position.
        position = newPosition;
    }
    if(mode == 1){
        Eigen::Vector2d initialHitDirection = hitPoint - position;
        if (initialHitDirection.norm() < stepSize) {
            position = hitPoint;
            goalQueue.clear();
            Eigen::Vector2d goal = helper.minDistance(environment.q_goal, waypoints);
            goalQueue = helper.shortestPath(position, goal, waypoints);
            mode = 2;
            return;
        }
    }
    if(mode == 2){
        if(goalQueue.empty()){
            goalQueue.push_back(environment.q_goal);
            mode = 0;
            return;
        }
        position = goalQueue.front();
        goalQueue.pop_front(); 
        return;
    }
    
};