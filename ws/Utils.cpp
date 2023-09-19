#include "Utils.h"

bool Utils::checkStep(Eigen::Vector2d start, Eigen::Vector2d stop, const amp::Problem2D& problem) {
    // Iterate through each obstacle, then each line segment, to check if the bug
    // motion from `start` to `stop` intersects those segments

    // Get all Obstacles
    for(amp::Obstacle2D obst : problem.obstacles){
        //Get all vertices
        std::vector<Eigen::Vector2d>& vertices = obst.verticesCCW();

        for(int i = 0; i < vertices.size(); i++){
            int j = (i == vertices.size() - 1) ? 0 : i + 1;

            if (Utils::checkLineSegmentIntersect(start, stop, vertices[i], vertices[j])) {
                return true;
            }
        }
    }

    return false; 
}

bool Utils::checkLineSegmentIntersect(Eigen::Vector2d start, Eigen::Vector2d stop,
        Eigen::Vector2d obsStart, Eigen::Vector2d obsStop) {
    // Basic line segment intersection check. If intersection occurs between
    // 0 and 1 for both segments, there was an intersection

    double t = (start(0) - obsStart(0)) * (obsStart(1) - obsStop(1)) - 
        (start(1) - obsStart(1)) * (obsStart(0) - obsStop(0));
    
    t /= (start(0) - stop(0)) * (obsStart(1) - obsStop(1)) - 
        (start(1) - stop(1)) * (obsStart(0) - obsStop(0));

    if (t < 0.0 || t > 1.0) {
        return false;
    }

    t = (start(0) - obsStart(0)) * (start(1) - stop(1)) - 
        (start(1) - obsStart(1)) * (start(0) - stop(0));
    
    t /= (start(0) - stop(0)) * (obsStart(1) - obsStop(1)) - 
        (start(1) - stop(1)) * (obsStart(0) - obsStop(0));

    if (t < 0.0 || t > 1.0) {
        return false;
    }

    return true;
}

Eigen::Vector2d Utils::rotateVec(Eigen::Vector2d vector, double angle) {
    //angle in degrees, translate to radians
    double rad = angle * M_PI / 180;
    
    Eigen::Matrix2d R;
    double c = cos(rad);
    double s = sin(rad);
    R(0,0) = c;
    R(1,1) = c;
    R(0,1) = -s;
    R(1,0) = s;

    return R * vector; //TODO: Is this actuall Vector2d?
}