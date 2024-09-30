#include "MyBugAlgorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {


    amp::Path2D path;
    
    path.waypoints.push_back(problem.q_init);

    for (int i = 0; i < 50; i = i + 1) {

        Eigen::Vector2d nextstep;
        nextstep << problem.q_init(0) + i*.02*(problem.q_goal(0) - problem.q_init(0)), problem.q_init(1) + i*.02*(problem.q_goal(1) - problem.q_init(1));
        
        if (PointInPolygon(nextstep, problem) == true){ //if the next step we want to take is in a collision
            // amp::Obstacle2D currentobs = CollisionWithObs(nextstep, problem);
            // path = followObs2(nextstep, problem, path, currentobs);

            path = followObstacle(nextstep, problem, path);
        }
        else{
            path.waypoints.push_back(nextstep);
        }
        
    }
    path.waypoints.push_back(problem.q_goal);

    
    

    return path;
}

double MyBugAlgorithm::Radians(double degrees){
    return degrees * (3.141592653589793238463/180);
}

double MyBugAlgorithm::DistToGoal(Eigen::Vector2d point1, const amp::Problem2D& problem){

    return sqrt((pow(point1[1] - double(problem.q_goal(1)),2)+(pow((point1[0] - problem.q_goal(0)),2))));
}

double MyBugAlgorithm::Dist(Eigen::Vector2d point1, Eigen::Vector2d point2){

    return sqrt((pow((point1[1] - point2[1]),2))+(pow((point1[0] - point2[1]),2)));
}

amp::Path2D MyBugAlgorithm::followObs2(Eigen::Vector2d point, const amp::Problem2D& problem, amp::Path2D path, amp::Obstacle2D obs){
    //CURRENTLY NOT IN USE
    std::vector<Eigen::Vector2d> vertices = obs.verticesCW();
    //find closest vertex
    int closest = 0;
    for (int i = 1; i < vertices.size(); i++){
        if (Dist(vertices[i], point) < Dist(vertices[closest], point)){
            closest = i;
        }
    }
    //go to that vrtex
    Eigen::Vector2d nextstep;
    for (int i = 0; i < 10; i++){
        nextstep << point[0] + i*.1*(vertices[closest][0] - point[0]), point[1] + i*.1*(vertices[closest][1] - point[1]);
        path.waypoints.push_back(nextstep);
    }

    point = path.waypoints[path.length()-1];
    std::cout<< "point changed to : " << point[0] << " , " << point[1] << "\n";

    for (int i = 0; i < vertices.size(); i++){ 
        int goal = (i + closest) % vertices.size(); //aim for the next vertex
        for (int j = 0; j < 10; j++){
            nextstep << point[0] + j*.1*(vertices[i][0] - point[0]), point[1] + j*.1*(vertices[i][1] - point[1]);
            path.waypoints.push_back(nextstep);
            
        }
        std::cout<< "point changed to : " << point[0] << " , " << point[1] << "\n";
        point = path.waypoints[path.length() -1];
    }

    return path;
}



amp::Path2D MyBugAlgorithm::followObstacle(Eigen::Vector2d point, const amp::Problem2D& problem, amp::Path2D path){
    //propose a point, 10 degrees rotated from our original proposal, so it shouldn't be on the m-line
    int angle = 0;
    Eigen::Vector2d nextstep; 
    double dist = DistToGoal(point, problem);
    nextstep << point[0] + .1*(problem.q_goal(0) - point[0])/dist, point[1] + .1*(problem.q_goal(1) - point[1])/dist;
    //also divide by the distance between them. get the euclidan distanee and divide.
    int ct = 0;
    while (PointInPolygon(nextstep, problem)){
        ct++;
        angle = (angle + 10) % 360;

        float newx = cos(Radians(angle)) * (nextstep[0]-point[0]) - sin(Radians(angle)) * (nextstep[1]-point[1]) + point[0];
        float newy = sin(Radians(angle)) * (nextstep[1]-point[1]) + cos(Radians(angle)) * (nextstep[1]-point[1]) + point[1];
        nextstep[0] = newx;
        nextstep[1] = newy;

        // iterate this until we get a safe point....not sure
        if (ct >= 20){break;}
    }
    path.waypoints.push_back(nextstep);
    //return path;

    //would be better practice if I just named it nextstep from the beginning
    
    int count = 0;

    point = nextstep; //discard our OG point, we're now at nextstep
    angle = 0; //reset angle

    while (!PointInMLine(point, problem)){ 
        //propose a point straight towards goal
        count++;
        //advance the next step
        double dist = DistToGoal(point, problem);
        nextstep << point[0] + .3*(problem.q_goal(0) - point[0])/dist, point[1] + .3*(problem.q_goal(1) - point[1])/dist; //now propose a new next step

        angle = 0;
        ct = 0;
        while (PointInPolygon(nextstep, problem)){
            ct++;
            //increment angle and re-propose
            float newx = cos(Radians(angle)) * (nextstep[0]-point[0]) - sin(Radians(angle)) * (nextstep[1]-point[1]) + point[0];
            float newy = sin(Radians(angle)) * (nextstep[1]-point[1]) + cos(Radians(angle)) * (nextstep[1]-point[1]) + point[1];
            nextstep[0] = newx;
            nextstep[1] = newy;
            angle = angle + 10;
            if (ct >= 15){break;}
        }
        path.waypoints.push_back(nextstep);
        point = nextstep;
        if (count == 100){
            break;
        }
    }
    return path;
}


bool MyBugAlgorithm::PointInMLine(Eigen::Vector2d point, const amp::Problem2D& problem) {
    double slope_from_init = (problem.q_goal(1) - problem.q_init(1))/(problem.q_goal(0) - problem.q_init(0));
    double slope_from_point = (problem.q_goal(1) - point[1])/(problem.q_goal(0) - point[0]);
    if ((slope_from_init == slope_from_point) && (point[0] >= fmin(problem.q_goal(0), problem.q_init(0))) && (point[0] <= fmax(problem.q_goal(0), problem.q_init(0)))){
        return true;
    }
    return false;
}


bool MyBugAlgorithm::PointInPolygon(Eigen::Vector2d point, const amp::Problem2D& problem) {
    for (amp::Obstacle2D obs : problem.obstacles){
        std::vector<Eigen::Vector2d> points = obs.verticesCCW();
        int i, j, nvert = points.size();
        bool c = false;
    
        for(i = 0, j = nvert - 1; i < nvert; j = i++) {
            if( ( (points[i][1] >= point[1] ) != (points[j][1] >= point[1]) ) &&
                (point[0] <= (points[j][0] - points[i][0]) * (point[1] - points[i][1]) / (points[j][1] - points[i][1]) + points[i][0])
            )
            c = !c;
        }
        if (c){
            return true;
        }
    }
    return false; //returns true if point is in polygon
}





amp::Obstacle2D MyBugAlgorithm::CollisionWithObs(Eigen::Vector2d point, const amp::Problem2D& problem){
    //CURRENTLY NOT IN USE
    for (amp::Obstacle2D obs : problem.obstacles){
        std::vector<Eigen::Vector2d> points = obs.verticesCCW();
        int i, j, nvert = points.size();
        bool c = false;
    
        for(i = 0, j = nvert - 1; i < nvert; j = i++) {
            if( ( (points[i][1] >= point[1] ) != (points[j][1] >= point[1]) ) &&
                (point[0] <= (points[j][0] - points[i][0]) * (point[1] - points[i][1]) / (points[j][1] - points[i][1]) + points[i][0])
            )
            c = !c;
        }
        if (c){
            //std::cout << "FOUND COLL" << "\n";
            return obs;
        }
    }
    return problem.obstacles[0];

}

//GRAVEYARD

// bool MyBugAlgorithm::inCollision(Eigen::Vector2d position,  std::vector<std::vector<int>> obs_primitives, const amp::Problem2D& problem){
//     for (int i = 0; i < problem.obstacles.size(); i++){ //for each obstacle
//         std::vector<int> prims = obs_primitives[i]; //all of the primitives for this obstacle
//         bool collisionposs = true;
//         for (int j = 0; j < problem.obstacles[i].verticesCCW().size(); j++){ //for each vertex in the obstacle
    
//             //i is obst, j is vertex
//             int thisprim = prims[j];
//             bool obsabove = (thisprim == 1); //means rest of obstacle is greater than the line describing the primitive
//             Eigen::Vector2d current = problem.obstacles[i].verticesCCW()[j]; //we will draw a line from previous to current
//             Eigen::Vector2d previous = problem.obstacles[i].verticesCCW()[j-1];
//             //std::cout << "vertices : " << previous[0] << ", " << previous[1] << ", " << current[0] << ", " << current[1] << "\n";

//             bool posabove;
//             if (position[1] > ((previous[1]-current[1])/(previous[0]-current[0]))*(position[0] - current[0]) + current[1]){
//                 posabove = 1;
//             }
//             else{ posabove = 0;}
//             if (posabove != obsabove){
//                 std::cout << " failed at vertices : " << previous[0] << ", " << previous[1] << ", " << current[0] << ", " << current[1];
//                 collisionposs = false;
//                 break;
//             }
//         }
//         if (collisionposs) {
//             std::cout << " COLLISION AT " << position[0] << " , " << position[1] << "\n";
//             //std::cout << "vertices : " << previous[0] << ", " << previous[1] << ", " << current[0] << ", " << current[1] << "\n";
//             return true;
//         }
//     }
//     //at this point, it has failed at every obstacle
//     return false;



// for (int i = 0; i < problem.obstacles.size(); i++){ //iterate through obstacles
//         std::vector<int> prims;
//         for (int j = 0; j < problem.obstacles[i].verticesCCW().size(); j++){ //iterate through vertices of that obstacle
//             Eigen::Vector2d current = problem.obstacles[i].verticesCCW()[j]; //we will draw a line from previous to current
//             Eigen::Vector2d previous = problem.obstacles[i].verticesCCW()[j-1];
//             Eigen::Vector2d comparison = problem.obstacles[i].verticesCCW()[j-2];

//             if (comparison[1] > ((previous[1]-current[1])/(previous[0]-current[0]))*(comparison[0] - current[0]) + current[1]){ //we're going from previous to current
//                 //obstacle is above line
//                 prims.push_back(1);
//             }
//             else{
//                 prims.push_back(-1);
//             }
//         }
//         obs_primitives.push_back(prims);
//     }
    //std::cout << "pushing back a list of size : " << obs_primitives.size() << " and obstacles are size " << problem.obstacles.size();

//should return true if it's on the correct side of every primitive in one obstacle 
//if it's not on the correct side of EVERY one, aka if it's on the wrong side of even one of them, move onto the next obstacle
//then return false if it fails every obstacle