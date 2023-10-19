#include "MyBugAlgorithm.h"
#include<iostream>
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    bugXY = problem.q_init;
    /*Eigen::Vector2d startPoint;
    startPoint << 2.5,2.5;
    startPoint << 6.5,3.5;
    bugXY = startPoint;*/
    bugXY = problem.q_init;
    path.waypoints.push_back(problem.q_init);
    hit = false;
    vertIdx = 0;
    qLScore = (bugXY - problem.q_goal).norm();
    double tempScore = qLScore;
    while(true){
        while(true){
            followMode = false;
            if(bugXY == problem.q_goal){
                path.waypoints.push_back(problem.q_goal);
                std::cout << "Success!" << std::endl;
                return path;
            }
            // make preliminary step
            bugNext = getNext(bugXY,problem.q_goal,2*bugStep);
            //For every object check if the next step will cause a collision
            stepCheck(bugXY,bugNext,problem);//hit set to false at start of stepCheck
            if(hit){
                //std::cout << "Hit an object, bugXY: " << bugXY << std::endl;
                //path = followBug1(problem, path);
                tempScore = (bugXY - problem.q_goal).norm();
                kill = false;
                path = followBug2(problem, path);
                //std::cout << "Leaving an object, bugXY: " << bugXY << std::endl;
                if(path.waypoints.back() == problem.q_init || (bugXY - problem.q_goal).norm() >= tempScore|| kill){
                //if(kill){
                    std::cout << "Killing because: b) " << ((bugXY - problem.q_goal).norm() >= tempScore) << " c) " << kill << std::endl;
                    //path.waypoints.push_back(problem.q_goal);
                    return path;
                }
            }
            else{
                //Continue toward goal
                if((problem.q_goal - bugXY).norm() < bugStep){
                    bugNext = getNext(bugXY,problem.q_goal,(problem.q_goal - bugXY).norm());
                }
                else{
                    bugNext = getNext(bugXY,problem.q_goal,bugStep);
                }
                path.waypoints.push_back(bugNext);
                bugXY = bugNext;
                //std::cout << "bugXY: " << bugXY << std::endl;
            }
            
        }
        
    }
    return path;
}