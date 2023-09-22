#include "MyBugAlgorithm.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    PRINT_VEC2("Start at ", problem.q_init);
    PRINT_VEC2("GOTO ", problem.q_goal);
    // Initialize variables
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    
    boundaryFollowing = 0;

    curDir = (problem.q_goal - problem.q_init).normalized();
    curGoal = problem.q_goal;
    
    while ((path.waypoints.back() - problem.q_goal).norm() > stepSize/100) {
        //DEBUG("Running Step" << path.waypoints.size());
        if (bugType == 1){
            path.waypoints.push_back(step(path.waypoints, problem));
        } else {
            path.waypoints.push_back(stepBug2(path.waypoints, problem));
        }
        //PRINT_VEC2("Adding point: ", path.waypoints.back());

        if (path.waypoints.size() > maxSteps) {
            DEBUG("Max Steps Reached");
            break;
        }
    }
    
    DEBUG("End");
    path.waypoints.push_back(problem.q_goal);

    /*
    
    Bug 2 needs a computeMLine fn
    Bug 2 probably needs a "follow obstacle to m_line" function
    
    Swap out the main file for something that tests components as I make them.

    For bug 2, need to check forward steps to see if m-line is crossed && distance is closer than m-line
    If yes, continue to goal
    If it gets back to hit point, exit with error
    */

    return path;
}


Eigen::Vector2d MyBugAlgorithm::step(std::vector<Eigen::Vector2d> path, 
    const amp::Problem2D& problem) {
    //First check to see if the step will overshoot the goal

    double distToGo = (curGoal - path.back()).norm();
    double useStepSize = (distToGo > stepSize) ? stepSize : distToGo;
    Eigen::Vector2d destination;
    int ktr = 0;

    // if not boundary following, try to take a step towards goal

    if (boundaryFollowing == 0){
        //DEBUG("Not Boundary Following");
        DEBUG(distToGo);
        destination = path.back() + useStepSize * curDir;

        if (!Utils::checkStep(path.back(), destination, problem)){
            return destination;
        } else {
            // Encountered a boundary, enter boundary following mode
            DEBUG("Encountered a boundary, enter boundary following mode");
            while (Utils::checkStep(path.back(), destination, problem)) {
                curDir = Utils::rotateVec(curDir, 1.0);
                destination = path.back() + useStepSize * curDir;
                ktr++;
                if (ktr > 190) {
                    DEBUG("overturned left");
                    throw std::logic_error("Turned all the way left, no valid path forward");
                }
            }
            
            boundaryFollowing = 1;
            hitPoint = path.back();
            boundaryTrace.push_back(path.back());
            boundaryDistances.push_back(distToGo);
            curGoal = path.back();
            return destination;
        }
    }

    // Respooling back to min distance location
    if (boundaryFollowing == 2) {
        //DEBUG("Respooling");
        Eigen::Vector2d spool = boundaryTrace.back();
        if (boundaryTrace.size() == 1) {
            //at min distance location, leave boundary following
            DEBUG("Finished respooling");
            curDir = (problem.q_goal - spool).normalized();
            boundaryFollowing = 0;
            boundaryTrace.clear();
            boundaryDistances.clear();
        } else {
            boundaryTrace.pop_back();
            boundaryDistances.pop_back();
        }
        return spool;
    }
    
    // Boundary Following
    // check that the obstacle is to the right
    //DEBUG("Boundary Following");
    Eigen::Vector2d rightHand = path.back() + 1.0 * stepSize * Utils::rotateVec(curDir, -90);
    ktr = 0;
    bool turned = false;
    double modifier = 2.0;
    while (!Utils::checkStep(path.back(), rightHand, problem)) {
        turned = true;
        curDir = Utils::rotateVec(curDir, -1);
        rightHand = path.back() + modifier * stepSize * Utils::rotateVec(curDir, -90);
        ktr++;
        if (ktr == 360) {
            modifier = 3.0; 
        }
        if (ktr > 720) {
            //DEBUG("Turned all the way right, lost obstacle");
            DEBUG("Boundary Following "<< boundaryFollowing);
            PRINT_VEC2("CurPos", path.back());
            //return problem.q_goal;
            throw std::logic_error("Turned all the way right, lost obstacle");
        }
    }
    if (turned) {
        //useStepSize *= .2;
        curDir = Utils::rotateVec(curDir, -90);
    }

    //check for obstacles ahead
    destination = path.back() + useStepSize * curDir;
    ktr = 0;
    while (Utils::checkStep(path.back(), destination, problem)) {
        curDir = Utils::rotateVec(curDir, 1);
        destination = path.back() + useStepSize * curDir;
        ktr++;
        if (ktr > 190) {
            //DEBUG("Turned all the way left, lost obstacle");
            throw std::logic_error("Turned all the way left, no valid path forward");
        }
    }

    // Check if back at hitPoint
    if ((curGoal - destination).norm() < 1.5*stepSize && boundaryDistances.size() > 3) {
        DEBUG("Back at hit point");
        boundaryFollowing = 2;
        curGoal = problem.q_goal;

        //DEBUG("Size of pre-reset spool is "<<boundaryDistances.size());

        // Check which direction to go back around
        auto it = std::min_element(boundaryDistances.begin(), boundaryDistances.end());
        int ind = std::distance(boundaryDistances.begin(), it);

        //DEBUG("Min Distance at ind " << ind << " with dist = " << boundaryDistances[ind]);

        // Reset spool to go back to min distance
        if (ind <= (boundaryDistances.size()/2)) {
            std::vector<double> tempDists = {boundaryDistances.begin(), boundaryDistances.begin() + ind};
            std::vector<Eigen::Vector2d> tempTrace = 
                {boundaryTrace.begin(), boundaryTrace.begin() + ind};

            std::reverse(tempDists.begin(), tempDists.end());
            std::reverse(tempTrace.begin(), tempTrace.end());

            boundaryDistances = tempDists;
            boundaryTrace = tempTrace;
            
        } else {
            std::vector<double> tempDists = {boundaryDistances.begin() + ind, boundaryDistances.end()};
            std::vector<Eigen::Vector2d> tempTrace = 
                {boundaryTrace.begin() + ind, boundaryTrace.end()};

            boundaryDistances = tempDists;
            boundaryTrace = tempTrace;
        }

        //DEBUG("Size of post-reset spool is "<<boundaryDistances.size());

        return hitPoint;
    }

    //take step    
    //DEBUG("Boundary Following");
    boundaryTrace.push_back(destination);
    boundaryDistances.push_back((problem.q_goal - destination).norm());
    return destination;
}

MyBugAlgorithm::MyBugAlgorithm() {
    bugType = 1;
    boundaryFollowing = 0;
    boundaryTrace.clear();
    boundaryDistances.clear();
    stepSize = .1;
    maxSteps = 10000;
}

Eigen::Vector2d MyBugAlgorithm::stepBug2(std::vector<Eigen::Vector2d> path, 
    const amp::Problem2D& problem) {
    //First check to see if the step will overshoot the goal

    double distToGo = (curGoal - path.back()).norm();
    double useStepSize = (distToGo > stepSize) ? stepSize : distToGo;
    Eigen::Vector2d destination;
    int ktr = 0;

    // if not boundary following, try to take a step towards goal

    if (boundaryFollowing == 0){
        //DEBUG("Not Boundary Following");
        DEBUG(distToGo);
        destination = path.back() + useStepSize * curDir;

        if (!Utils::checkStep(path.back(), destination, problem)){
            return destination;
        } else {
            // Encountered a boundary, enter boundary following mode
            DEBUG("Encountered a boundary, enter boundary following mode");
            while (Utils::checkStep(path.back(), destination, problem)) {
                curDir = Utils::rotateVec(curDir, 1.0);
                destination = path.back() + useStepSize * curDir;
                ktr++;
                if (ktr > 190) {
                    DEBUG("overturned left");
                    throw std::logic_error("Turned all the way left, no valid path forward");
                }
            }
            
            boundaryFollowing = 1;
            hitPoint = path.back();
            boundaryTrace.push_back(path.back());
            boundaryDistances.push_back(distToGo);
            curGoal = path.back();
            return destination;
        }
    }

    // Respooling back to min distance location
    if (boundaryFollowing == 2) {
        //DEBUG("Respooling");
        Eigen::Vector2d spool = boundaryTrace.back();
        if (boundaryTrace.size() == 1) {
            //at min distance location, leave boundary following
            DEBUG("Finished respooling");
            curDir = (problem.q_goal - spool).normalized();
            boundaryFollowing = 0;
            boundaryTrace.clear();
            boundaryDistances.clear();
        } else {
            boundaryTrace.pop_back();
            boundaryDistances.pop_back();
        }
        return spool;
    }
    
    // Boundary Following
    // check that the obstacle is to the right
    //DEBUG("Boundary Following");
    Eigen::Vector2d rightHand = path.back() + 1.0 * stepSize * Utils::rotateVec(curDir, -90);
    ktr = 0;
    bool turned = false;
    double modifier = 2.0;
    while (!Utils::checkStep(path.back(), rightHand, problem)) {
        turned = true;
        curDir = Utils::rotateVec(curDir, -1);
        rightHand = path.back() + modifier * stepSize * Utils::rotateVec(curDir, -90);
        ktr++;
        if (ktr == 360) {
            modifier = 3.0; 
        }
        if (ktr > 720) {
            //DEBUG("Turned all the way right, lost obstacle");
            DEBUG("Boundary Following "<< boundaryFollowing);
            PRINT_VEC2("CurPos", path.back());
            //return problem.q_goal;
            throw std::logic_error("Turned all the way right, lost obstacle");
        }
    }
    if (turned) {
        //useStepSize *= .2;
        curDir = Utils::rotateVec(curDir, -90);
    }

    //check for obstacles ahead
    destination = path.back() + useStepSize * curDir;
    ktr = 0;
    while (Utils::checkStep(path.back(), destination, problem)) {
        curDir = Utils::rotateVec(curDir, 1);
        destination = path.back() + useStepSize * curDir;
        ktr++;
        if (ktr > 190) {
            //DEBUG("Turned all the way left, lost obstacle");
            throw std::logic_error("Turned all the way left, no valid path forward");
        }
    }

    // Check if back at hitPoint
    if ((curGoal - destination).norm() < 1.5*stepSize && boundaryDistances.size() > 3) {
        DEBUG("Back at hit point");
        boundaryFollowing = 2;
        curGoal = problem.q_goal;

        //DEBUG("Size of pre-reset spool is "<<boundaryDistances.size());

        // Check which direction to go back around
        auto it = std::min_element(boundaryDistances.begin(), boundaryDistances.end());
        int ind = std::distance(boundaryDistances.begin(), it);

        //DEBUG("Min Distance at ind " << ind << " with dist = " << boundaryDistances[ind]);

        // Reset spool to go back to min distance
        if (ind <= (boundaryDistances.size()/2)) {
            std::vector<double> tempDists = {boundaryDistances.begin(), boundaryDistances.begin() + ind};
            std::vector<Eigen::Vector2d> tempTrace = 
                {boundaryTrace.begin(), boundaryTrace.begin() + ind};

            std::reverse(tempDists.begin(), tempDists.end());
            std::reverse(tempTrace.begin(), tempTrace.end());

            boundaryDistances = tempDists;
            boundaryTrace = tempTrace;
            
        } else {
            std::vector<double> tempDists = {boundaryDistances.begin() + ind, boundaryDistances.end()};
            std::vector<Eigen::Vector2d> tempTrace = 
                {boundaryTrace.begin() + ind, boundaryTrace.end()};

            boundaryDistances = tempDists;
            boundaryTrace = tempTrace;
        }

        //DEBUG("Size of post-reset spool is "<<boundaryDistances.size());

        return hitPoint;
    }

    //take step    
    //DEBUG("Boundary Following");
    boundaryTrace.push_back(destination);
    boundaryDistances.push_back((problem.q_goal - destination).norm());
    return destination;
}