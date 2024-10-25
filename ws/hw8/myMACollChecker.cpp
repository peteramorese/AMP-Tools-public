#include "myMACollChecker.h"

//idea: could have a function called getCoord(state, i) that returns the x and y coordinates of robot i
Eigen::Vector2d getCoord(const Eigen::VectorXd& state, int i) {
    Eigen::Vector2d result;
    result[0] = state[i * 2];
    result[1] = state[(i * 2) + 1];
    //std::cout<< "robot " << i << " is at " << result[0] << ", " << result[1] << "\n";
    return result;
}


bool MyMACollChecker::diskCollision(Eigen::Vector2d p1, Eigen::Vector2d p2) const{
    // std::cout << "checking collision between " << p1.transpose() << " and " << p2.transpose() << "\n";
    // std::cout << "distance between them is " << (p1 - p2).norm() << "\n";
    bool coll = (p1 - p2).norm() < robot_radius*cautious_radius;
    // std::cout << "collision? " << coll << "\n";

    return (p1 - p2).norm() < robot_radius*cautious_radius; //REPLACE WITH RADIUS
}

//could try using this circlePoly intersection.
bool circlePoly(amp::Obstacle2D obs, Eigen::Vector2d point, double radius){
    std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();
    for (int i = 0; i < vertices.size(); i++) {
        Eigen::Vector2d v1 = vertices[i];
        Eigen::Vector2d v2 = vertices[(i+1)%vertices.size()];
        Eigen::Vector2d edge = v2 - v1;
        Eigen::Vector2d pointToEdge = point - v1;
        double edgeLength = edge.norm();
        double edgeDotProduct = edge.dot(pointToEdge);
        if (edgeDotProduct >= 0 && edgeDotProduct <= edgeLength) {
            Eigen::Vector2d closestPoint = v1 + (edge * (edgeDotProduct / edgeLength));
            if ((point - closestPoint).norm() < radius*2) {
                return true;
            }
        }
    }
    return false;
}


bool MyMACollChecker::inCollision(amp::MultiAgentProblem2D problem, const Eigen::VectorXd& cspace_state) const {
    // std::cout << "checking collision" << "\n";
    // std::cout << "obstacles size: " << problem.obstacles.size() << "\n";
    
    int numAgents = problem.numAgents();
    for (int i = 0; i < numAgents; i++){ //first check robot-robot collision
        for (int j = i+1; j < numAgents; j++){
            // std::cout << "checking robot " << i << " and robot " << j << "\n";
            // std::cout << "checking coord " << getCoord(cspace_state, i) << " and vs " << getCoord(cspace_state, j) << "\n";
            
            if (diskCollision(getCoord(cspace_state, i), getCoord(cspace_state, j))){

                // std::cout << "robot collision at" << getCoord(cspace_state, i) << ", " << getCoord(cspace_state, j) << "\n";
                return true;
            }
        }
        // std::cout << "checking robot " << i << "\n";
    }
    bool testmode = false;
    if (testmode){
        return false;
    }
    for (int i = 0; i < numAgents; i++){
        Eigen::Vector2d point = getCoord(cspace_state, i); //THIS NEEDS TO BE FIXED; doesn't take the radius into account
        //ideas: can either discretize around the radius of the robot and check like 5-10 points... or smt else
        //can also do it the right way and just make a cspace grid
        for (amp::Obstacle2D obs : problem.obstacles){
            // std::cout<<"checking obstacle" << "\n";
            if (circlePoly(obs, point, robot_radius*cautious_radius)){
                // std::cout << "obstacle collision at" << point.transpose() << "\n";
                return true;
            }

        }
    }
    
    //now check robot-obstacle collision
    // for (int i = 0; i < numAgents; i++){
    //     Eigen::Vector2d point = getCoord(cspace_state, i); //THIS NEEDS TO BE FIXED; doesn't take the radius into account
    //     //ideas: can either discretize around the radius of the robot and check like 5-10 points... or smt else
    //     //can also do it the right way and just make a cspace grid
    //     for (amp::Obstacle2D obs : problem.obstacles){
    //         // std::cout<<"checking obstacle" << "\n";
    //         std::vector<Eigen::Vector2d> points = obs.verticesCCW();
    //         int k, m, nvert = points.size();
    //         bool c = false;
    //         for(k = 0, m = nvert - 1; k < nvert; m = k++) {
    //             if( ( (points[k][1] >= cspace_state[1] ) != (points[m][1] >= cspace_state[1]) ) &&
    //                 (cspace_state[0] <= (points[m][0] - points[k][0]) * (cspace_state[1] - points[k][1]) / (points[m][1] - points[k][1]) + points[k][0])
    //             )   
    //             c = !c;
    //             }
    //             if (c){
    //                 return true;
    //         }
    //     }
    // }
    return false;
}


bool MyMACollChecker::inCollision(amp::MultiAgentProblem2D problem, const Eigen::VectorXd& startpoint, const Eigen::VectorXd& endpoint) const{
    // std::cout << "checking collision between points: " << startpoint.transpose() << ", " << endpoint.transpose() << "\n";
    for (double t = 0; t <= 1; t+= 0.1){
        Eigen::VectorXd point = startpoint + (endpoint - startpoint) * t;
        if (inCollision(problem, point)){
            return true;
        }
    }
    return false;
}



//this is hthe dummy fake one

bool MyMACollChecker::inCollision(const Eigen::VectorXd& cspace_state) const {
    //DUPE
    // std::cout << "checking collision" << "\n";
    // std::cout << "obstacles size: " << myproblem.obstacles.size() << "\n";
        for (amp::Obstacle2D obs : myproblem.obstacles){
            // std::cout<<"checking obstacle" << "\n";
            std::vector<Eigen::Vector2d> points = obs.verticesCCW();
            int k, m, nvert = points.size();
            bool c = false;
            for(k = 0, m = nvert - 1; k < nvert; m = k++) {
                if( ( (points[k][1] >= cspace_state[1] ) != (points[m][1] >= cspace_state[1]) ) &&
                    (cspace_state[0] <= (points[m][0] - points[k][0]) * (cspace_state[1] - points[k][1]) / (points[m][1] - points[k][1]) + points[k][0])
                )   
                c = !c;
                }
                if (c){
                    return true;
            }
        }
        return false;
}