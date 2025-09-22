#include "utils.h"
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <limits>
#include <set>
#include <queue>
#include <unordered_set>
#include <iostream>

namespace utils{
    
    //helper function that acts as a sensor. Project a point in front of point, distance epsilon, direction theta. Check for obstacle hits. Return bool for hitpoint.
    bool sensor(double epsilon, double theta, std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d point) {
            //get the point in the direction of theta
            Eigen::Vector2d sensor_hit = point + epsilon * Eigen::Vector2d(std::cos(theta), std::sin(theta));
            //check if this is inside of an obstacle
            for(auto& obstacle : obstacles){
                if(raycastObstacleDetection(obstacle, sensor_hit)){
                    return true; //hit an obstacle
                }
            }
        return false;
    }

    //check if there is an intersect between the line segement p and q given the endpoints.
    bool checkIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2){
        //TODO: write this function
        Eigen::Vector2d directionp = p2 - p1;
            Eigen::Vector2d directionq = q2 - q1;
            Eigen::Vector2d qp = q1 - p1;
            double directionp_cross_directionq = directionp.x() * directionq.y() - directionp.y() * directionq.x();
            double qp_cross_directionp = qp.x() * directionp.y() - qp.y() * directionp.x();

            //check if parallel line (not just segment) 
            if(std::abs(directionp_cross_directionq) < 1e-10){
                return false;
            }

            //solve scalars for each vector that would give us the intersection point
            double t = (qp.x() * directionq.y() - qp.y() * directionq.x()) / directionp_cross_directionq;
            double u = qp_cross_directionp / directionp_cross_directionq;

            //make sure the scalars are between 0 and 1, i.e. the intersection point is between the two segments (not outside)
            if(t >= 0 && t<=1 && u>=0 && u<=1){
                return true;
            }
            return false;

    }

    //return the raytrace hitpoint from q_init to q_goal. If no hitpoint return q_goal.
    Eigen::Vector2d raytrace(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d q_init, Eigen::Vector2d q_goal, double epsilon) {
        //linear interpolation with stepsize epsilon from q_init to q_goal.
        std::vector<Eigen::Vector2d> interp;
        Eigen::Vector2d direction = q_goal - q_init;
        double distance = direction.norm();
        //if same point return
        if(distance == 0) {
            return q_goal;
        }

        direction = direction.normalized();
        int numSteps = static_cast<int>(std::ceil(distance / epsilon)) - 1; //-1 so that we don't do the last point, for that we will use q_goal as the last poitn might be past q_goal.

        for(int i = 0; i < numSteps; i++) {
            double stepDist = i * epsilon;
            interp.push_back(q_init + direction * stepDist);
        }

        //now we want to raymarch through the points and do collision detection for each of those points.
        for(int i = 0; i < numSteps; i++) {
            //for each obstacle check if we are inside it
            for(const auto& obstacle : obstacles) {
                if(raycastObstacleDetection(obstacle, interp[i]))
                    return interp[i];
            }
        }
        return q_goal;

    }

    //cehck if this point is inside the obstacle by checking how many intersections a raycast to the right has. If even, we are outside the obstacle. If odd, we are inside. Check intersects for each edge (doesn't use intersect function as it was written before)
    bool raycastObstacleDetection(const amp::Obstacle2D& obstacle, const Eigen::Vector2d& point) {
        bool inside = false;
        int numV = obstacle.verticesCCW().size();
        for(int i = 0, j = numV-1; i < numV; j = i++){
            const Eigen::Vector2d& pi = obstacle.verticesCCW()[i];
            const Eigen::Vector2d& pj = obstacle.verticesCCW()[j];

            //now check for intersects when casting to the right
            //if the line is horizontal, no intersect so stay same
            if(pi.y() == pj.y()) {
                continue;
            }
            //otherwise check if our poitns y is between these two
            bool yBetween = (point.y() > std::min(pi.y(), pj.y())) && (point.y() <= std::max(pi.y(), pj.y()));
            if (yBetween) {
                //get x coordinate of intersect
                double t = (point.y() - pi.y()) / (pj.y() - pi.y());  
                double intersectX = pi.x() + t * (pj.x() - pi.x());
                if (intersectX > point.x()){
                    //flip inside
                    inside = !inside;
                }
            }
        }
        return inside;
    }
  
};