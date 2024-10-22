#include "hw7helpers.h"
#include "AMPCore.h"

/// @brief Compute the Euclidean distance between two states in configuration space
/// @param lhs First state
/// @param rhs Second state
/// @return Euclidean distance between the two states
double DistanceMetric::distance(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) {
    return (lhs - rhs).norm();
}


bool MyPointCollisionChecker::inCollision(amp::Problem2D problem, const Eigen::VectorXd& startpoint, const Eigen::VectorXd& endpoint) const{
    // std::cout << "checking collision between points: " << startpoint.transpose() << ", " << endpoint.transpose() << "\n";
    for (double t = 0; t <= 1; t+= 0.05){
        
        for (amp::Obstacle2D obs : problem.obstacles){
                        std::vector<Eigen::Vector2d> points = obs.verticesCCW();
                        int k, m, nvert = points.size();
                        bool c = false;
                        //Eigen::Vector2d point = startpoint + (endpoint - startpoint) * t / (endpoint[0] - startpoint[0]);
                        // Eigen::Vector2d point = Eigen::Vector2d{t, startpoint[1] + ((endpoint[1] - startpoint[1]) * t / (endpoint[0] - startpoint[0]))} ;
                        Eigen::VectorXd point = startpoint + (endpoint - startpoint) * t;
                        // std::cout << "point: " << point.transpose() << "\n";
                        for(k = 0, m = nvert - 1; k < nvert; m = k++) {
                            if( ( (points[k][1] >= point[1] ) != (points[m][1] >= point[1]) ) &&
                                (point[0] <= (points[m][0] - points[k][0]) * (point[1] - points[k][1]) / (points[m][1] - points[k][1]) + points[k][0])
                            )   
                            c = !c;
                            }
                            if (c){
                                // std::cout<<"collision found" << "\n";
                                return true;
                            }
                    }
    }
    return false;
}

/// @brief Check if a state in configuration space is colliding with an obstacle
/// @param cspace_state N-dimensional C-space state
/// @return `true` if the the state is in collision, `false` if it is not
bool MyPointCollisionChecker::inCollision(amp::Problem2D problem, const Eigen::VectorXd& cspace_state) const {
    // std::cout << "checking collision" << "\n";
    // std::cout << "obstacles size: " << problem.obstacles.size() << "\n";
        for (amp::Obstacle2D obs : problem.obstacles){
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

//this is hthe dummy fake one

bool MyPointCollisionChecker::inCollision(const Eigen::VectorXd& cspace_state) const {
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


