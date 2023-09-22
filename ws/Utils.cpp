#include "Utils.h"

// Return true if the possible step intersects an obstacle
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
                //DEBUG("Found Hit");
                //PRINT_VEC2("V1", vertices[i]);
                //PRINT_VEC2("V2", vertices[j]);
                return true;
            }
        }
    }

    return false; 
}

bool Utils::checkLineSegmentIntersect(Eigen::Vector2d start, Eigen::Vector2d stop,
        Eigen::Vector2d obsStart, Eigen::Vector2d obsStop) {

    //if parallel, check for colinear
    //if no, return false
    //if yes, check if end points are within the other line segment

    double denom = (start(0) - stop(0)) * (obsStart(1) - obsStop(1)) - 
        (start(1) - stop(1)) * (obsStart(0) - obsStop(0));

    //DEBUG(denom);
    if (denom == 0) {
        //Parallel, check for collinear and overlap
        //just in case, check all 4 cases
        return checkCollinearOverlap(start(0), start(1), stop(0), stop(1), obsStart(0), obsStart(1)) ||
            checkCollinearOverlap(start(0), start(1), stop(0), stop(1), obsStop(0), obsStop(1)) ||
            checkCollinearOverlap(obsStart(0), obsStart(1), obsStop(0), obsStop(1), start(0), start(1)) ||
            checkCollinearOverlap(obsStart(0), obsStart(1), obsStop(0), obsStop(1), stop(0), stop(1));
    }

    // Basic line segment intersection check. If intersection occurs between
    // 0 and 1 for both segments, there was an intersection

    double t = (start(0) - obsStart(0)) * (obsStart(1) - obsStop(1)) - 
        (start(1) - obsStart(1)) * (obsStart(0) - obsStop(0));
    t /= denom;
    //DEBUG(t);
    if (t < 0.0 || t > 1) {
        return false;
    }

    t = (start(0) - obsStart(0)) * (start(1) - stop(1)) - 
        (start(1) - obsStart(1)) * (start(0) - stop(0));
    t /= denom;
    //DEBUG(t);
    if (t < 0.0 || t > 1) {
        return false;
    }

    return true;
}

bool Utils::checkCollinearOverlap(double x1, double y1, double x2, double y2, 
        double x3, double y3) {
    //shoelace formula to check all three points are collinear
    //true if area of triangle = 0

    double area2 = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);

    if (abs(area2) > .0000001) {
        //not collinear
        return false;
    }

    //If collinear, check if point 3 is bounded by points 1 and 2

    return (x3 <= std::max(x1, x2) && x3 >= std::min(x1, x2) && 
        y3 <= std::max(y1, y2) && y3 >= std::min(y1, y2));

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