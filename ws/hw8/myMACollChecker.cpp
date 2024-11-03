#include "myMACollChecker.h"

//idea: could have a function called getCoord(state, i) that returns the x and y coordinates of robot i
Eigen::Vector2d getCoord(const Eigen::VectorXd& state, int i) {
    Eigen::Vector2d result;
    result[0] = state[i * 2];
    result[1] = state[(i * 2) + 1];
    //std::cout<< "robot " << i << " is at " << result[0] << ", " << result[1] << "\n";
    return result;
}


bool MyMACollChecker::diskCollision(Eigen::Vector2d p1, Eigen::Vector2d p2, double radius) const{
    // std::cout << "checking collision between " << p1.transpose() << " and " << p2.transpose() << "\n";
    // std::cout << "distance between them is " << (p1 - p2).norm() << "\n";
    bool coll = (p1 - p2).norm() < radius*cautious_radius;
    // std::cout << "collision? " << coll << "\n";

    return (p1 - p2).norm() < radius*cautious_radius; //REPLACE WITH RADIUS
}

//could try using this circlePoly intersection.
bool MyMACollChecker::circlePoly(amp::Obstacle2D obs, Eigen::Vector2d point, double radius) const {
    // std::cout << "working with radius: " << radius << "\n";
    std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();
    for (int i = 0; i < vertices.size(); i++) {
        Eigen::Vector2d v1 = vertices[i];
        bool coll = diskCollision(point, v1, radius*2);
        if (coll) { 
            // std::cout << "vertex collision at" << point.transpose() << "\n";
            return true;
        }
    }
    for (int i = 0; i < vertices.size(); i++) {
        Eigen::Vector2d v1 = vertices[i];
        Eigen::Vector2d v2 = vertices[(i+1)%vertices.size()];
        Eigen::Vector2d edge = v2 - v1;
        Eigen::Vector2d pointToEdge = point - v1;
        double edgeLength = edge.norm();
        double edgeDotProduct = edge.dot(pointToEdge);
        if (edgeDotProduct >= 0 && edgeDotProduct <= edgeLength) {
            Eigen::Vector2d closestPoint = v1 + (edge * (edgeDotProduct / edgeLength));
            if ((point - closestPoint).norm() < radius*cautious_radius) {
                // std::cout << "circlePoly collision at" << point.transpose() << "\n";
                return true;
            }
        }
    }
    std::vector<Eigen::Vector2d> pointsOnCircle;
    for (int i = 0; i < 10; i++) {
        double theta = 2 * 3.14159 * i / 10;
        Eigen::Vector2d p = point + Eigen::Vector2d{radius * cos(theta), radius * sin(theta)};
        pointsOnCircle.push_back(p);
    }
    pointsOnCircle.push_back(point);
    for (int i = 0; i < pointsOnCircle.size(); i++) {
        Eigen::Vector2d pt = pointsOnCircle[i];
        std::vector<Eigen::Vector2d> points = obs.verticesCCW();
        int k, m, nvert = points.size();
        bool c = false;
        //Eigen::Vector2d point = startpoint + (endpoint - startpoint) * t / (endpoint[0] - startpoint[0]);
        // Eigen::Vector2d point = Eigen::Vector2d{t, startpoint[1] + ((endpoint[1] - startpoint[1]) * t / (endpoint[0] - startpoint[0]))} ;
        // std::cout << "point: " << point.transpose() << "\n";
        for(k = 0, m = nvert - 1; k < nvert; m = k++) {
            if( ( (points[k][1] >= pt[1] ) != (points[m][1] >= pt[1]) ) &&
                (pt[0] <= (points[m][0] - points[k][0]) * (pt[1] - points[k][1]) / (points[m][1] - points[k][1]) + points[k][0])
            )   
            c = !c;
            }
            if (c){
                // std::cout<<"collision found" << "\n";
                return true;
            }
        }
    

    
    return false;
}


bool MyMACollChecker::inCollision(amp::MultiAgentProblem2D problem, const Eigen::VectorXd& cspace_state) const {
    // std::cout << "checking collision" << "\n";
    // std::cout << "obstacles size: " << problem.obstacles.size() << "\n";
    
    int numAgents = cspace_state.size()/2;
    for (int i = 0; i < numAgents; i++){ //first check robot-robot collision
        for (int j = i+1; j < numAgents; j++){
            // std::cout << "checking robot " << i << " and robot " << j << "\n";
            // std::cout << "checking coord " << getCoord(cspace_state, i) << " and vs " << getCoord(cspace_state, j) << "\n";
            double radius = get_agent_radius(i) + get_agent_radius(j);
            if (diskCollision(getCoord(cspace_state, i), getCoord(cspace_state, j), radius*1.05)){

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
            double rob_radius = get_agent_radius(i);
            if (circlePoly(obs, point, rob_radius*cautious_radius*1.05)){
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
    for (double t = .05; t <= 1; t+= 0.1){
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