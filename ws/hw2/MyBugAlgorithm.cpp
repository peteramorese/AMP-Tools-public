#include "MyBugAlgorithm.h"
#include <iostream>
#include "utils.h"
#include <cmath>

// Implement your methods in the `.cpp` file, for example:

//NOTE: EXACTLY ONE OF THESE MUST BE DEFINED FOR THE CODE TO COMPILE.
// #define BUG1
#define BUG2





amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    double epsilon = 1e-3;
    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    Eigen::Vector2d curr_point = problem.q_init;

    //count variable to make sure we don't spin in place.
    int count = 0;
    //direction to goal
    Eigen::Vector2d direction = problem.q_goal - problem.q_init;
    //heading angle
    double theta = std::atan2(direction.y(), direction.x());
    //push back the initial point
    path.waypoints.push_back(problem.q_init);
    //while not close to goal
    while((curr_point - problem.q_goal).norm() > 2 * epsilon){
        //if no obstacle in front, move forwards
        if(!utils::sensor(epsilon, theta, problem.obstacles, curr_point)){
            curr_point = curr_point + epsilon * Eigen::Vector2d(std::cos(theta), std::sin(theta));
        }
        //else, wall follow mode!
        else{
            //push back the current point to make sure we record it.
            path.waypoints.push_back(curr_point);
            //record some variables...
            //start wall follow position
            Eigen::Vector2d start_point = curr_point;
            //closest point for BUG1
            Eigen::Vector2d closest_point = curr_point;
            //dist to goal for BUG1
            double dist_to_goal = (curr_point - problem.q_goal).norm();
            //bool to make sure we go past our starting point a little...
            bool started_moving = false;
            //number of points in the path to the closest point for BUG1
            int num_path_points = path.waypoints.size();
            //the number of movements we've made (to increment started moving)
            int move_count = 0;

            //MACRO defining how we compile our code. Different compilation behavior for BUG1 and BUG2.
            #ifdef BUG1
                //If BUG1, we need to run until we started moving and we are within epsilon of the starting point.
                while(!started_moving || (curr_point - start_point).norm() > epsilon){
            #endif
            #ifdef BUG2
                //If BUG2, Check that we started moving, and continue as long as our last point in the path and current point don't intersect our start point and goal point.
                while(!started_moving  || !utils::checkIntersect(path.waypoints[path.waypoints.size()-1], curr_point, start_point, problem.q_goal) || (start_point - problem.q_goal).norm() < (curr_point - problem.q_goal).norm()){
            #endif

                //increment count to make sure we don't spin in circles forever
                count++;
                if(count > 599990){
                    std::cout << "hit wall count " << theta << std::endl;
                }
                if(count > 600000){
                    
                    return path;
                }
                
                //if the front sensor sees the obstacle
                if(utils::sensor(epsilon, theta, problem.obstacles, curr_point)){
                    //record theta and spin in place until we don't see an obstacle.
                    double old_theta = theta;
                    while(utils::sensor(epsilon, theta,     problem.obstacles, curr_point)){
                    theta = theta + epsilon;
                    //if surrounded by obstacles bad, how did we get here? just return
                    if(abs(old_theta - theta) > 360){
                        //bad
                        return path;
                    }
                    }
                    //push back current point
                    path.waypoints.push_back(curr_point);
                    //increment current point. no obstacle in front
                    curr_point = curr_point + epsilon/5 * Eigen::Vector2d(std::cos(theta), std::sin(theta));
                    double dist = (curr_point - problem.q_goal).norm();
                    //update dist_to_goal and curr_point for BUG1.
                    if(dist < dist_to_goal){
                        dist_to_goal = dist;
                        closest_point = curr_point;
                    }
                } 
                //if the pi/4 doesn't see the obstacle, turn right
                else if(!utils::sensor(epsilon * 1.43, theta-3.1415/4, problem.obstacles, curr_point)){
                    theta = theta - epsilon/2;
                }
                //if pi/2 sensor doesn't see obstacle, turn right a lot, move forwards a little (the obstacle should be there), and then turn back
                else if(!utils::sensor(epsilon, theta-3.1415/2, problem.obstacles, curr_point)){
                    theta = theta - 3.1415/2;
                    path.waypoints.push_back(curr_point);
                    curr_point = curr_point + epsilon/5 * Eigen::Vector2d(std::cos(theta), std::sin(theta)); 
                    //update dist to goal and closest point for BUG1
                    double dist = (curr_point - problem.q_goal).norm();
                    if(dist < dist_to_goal){
                        dist_to_goal = dist;
                        closest_point = curr_point;
                    }
                    //update theta to be slightly more turned right
                    theta = theta + 3.1415/2 - epsilon;
                }
                else{
                    //else we see obstacle to right, but not in front. we can move forward!!
                    //update
                    path.waypoints.push_back(curr_point);
                    curr_point = curr_point + epsilon * Eigen::Vector2d(std::cos(theta), std::sin(theta));
                    //check if we have moved enough, if so let us get out of while loop
                    if(move_count < 5 && !started_moving){
                        move_count ++;
                        started_moving = true;
                    }
                    else if(move_count < 5){
                        move_count ++;
                    }
                    
                    //check dist to goal for BUG1
                    double dist = (curr_point - problem.q_goal).norm();
                    if(dist < dist_to_goal){
                        dist_to_goal = dist;
                        closest_point = curr_point;
                    }
                }
            }

            //if we are running bug1, we need to go back to the closest point.
            #ifdef BUG1
                //go back to the closest point.
                while((curr_point - problem.q_goal).norm() != dist_to_goal){
                    curr_point = path.waypoints[num_path_points];
                    path.waypoints.push_back(curr_point);
                    num_path_points++;

                }
            #endif
            //reset theta to go towards goal.
            direction = problem.q_goal - curr_point;
            theta = std::atan2(direction.y(), direction.x());
            path.waypoints.push_back(curr_point);
        }
    }

    //close enough to goal and no obstacles, go there and return path
    path.waypoints.push_back(problem.q_goal);
    return path;
}
