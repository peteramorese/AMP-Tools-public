// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
#include <cmath>

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;




amp::Polygon makeCSpaceObstacle(amp::Polygon robot, amp::Polygon obstacle){
    //iterate through the robot points
    std::vector<Eigen::Vector2d> robotpts = robot.verticesCCW();
    std::vector<Eigen::Vector2d> obspts = obstacle.verticesCCW();
    std::vector<Eigen::Vector2d> cobspts;
    std::vector<int> obsverts = {0,0,1,1,2,2};
    std::vector<int> robverts = {1,2,2,0,0,1};
    for (int i = 0; i < 6; i++){
        Eigen::Vector2d vertex;
        vertex << obspts[obsverts[i]][0] - robotpts[robverts[i]][0], obspts[obsverts[i]][1] - robotpts[robverts[i]][1];
        cobspts.push_back(vertex);
    }
    amp::Polygon result = Polygon(cobspts);

    return result;   

}

std::vector<amp::Polygon> makeCSpaceObstacleRot(amp::Polygon robot, amp::Polygon obstacle){
    //iterate through the robot points
    std::vector<amp::Polygon> result;
    std::vector<Eigen::Vector2d> robotpts = robot.verticesCCW();
    std::vector<Eigen::Vector2d> obspts = obstacle.verticesCCW();
    
    std::vector<int> obsverts = {0,0,1,1,2,2};
    std::vector<int> robverts = {1,2,2,0,0,1};
    for (double k = 0; k < 10; k++){
        //std::cout<<"j: " << j << "\n";
        double j = 2*M_PI / 10;
        std::vector<Eigen::Vector2d> cobspts;
        for (int i = 0; i < robotpts.size(); i++){ //rotate 
            double newx = robotpts[i][0]*cos(j) - robotpts[i][1]*sin(j);
            double newy = robotpts[i][0]*sin(j) + robotpts[i][1]*cos(j);
            robotpts[i][0] = newx;
            robotpts[i][1] = newy;
        }

        for (int i = 0; i < 6; i++){
            Eigen::Vector2d vertex;
            vertex << obspts[obsverts[i]][0] - robotpts[robverts[i]][0], obspts[obsverts[i]][1] - robotpts[robverts[i]][1];
            cobspts.push_back(vertex);
        }
        amp::Polygon poly = Polygon(cobspts);
        result.push_back(poly);
    }
    return result;

}

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    // amp::RNG::seed(amp::RNG::randiUnbounded());
    std::vector<double> link_lengths = {1, 1, 1};
    MyManipulator2D manipulator(link_lengths);
    std::vector<amp::Polygon> WSobs = HW4::getEx3Workspace3().obstacles;
    Visualizer::makeFigure(WSobs);  // Type of polygons: std::vector<amp::Polygon>
    // std::vector<Eigen::Vector2d> ex1obs= HW4::getEx1TriangleObstacle().verticesCCW();
    // std::vector<amp::Polygon> cobs = makeCSpaceObstacleRot(ex1obs, ex1obs);
    // amp::Polygon cobs1 = makeCSpaceObstacle(ex1obs, ex1obs);
    // std::vector<amp::Polygon> polygons;
    // std::vector<double> angles;
    // for (double k = 0; k < 10; k++){
    //     //std::cout<<"j: " << j << "\n";
    //     double j = 2*M_PI / 10;
    //     angles.push_back(k * 2 * M_PI / 10);
    //     polygons.push_back(cobs1);

    // }
    // //polygons.push_back(cobs);
    // Visualizer::makeFigure(polygons, angles );  // Type of polygons: std::vector<amp::Polygon>
    // for (Eigen::Vector2d vertex : ex1obs){
    //     std::cout << vertex[0] << vertex[1];
    // }

    // You can visualize your manipulator given an angle state like so:
    amp::ManipulatorState test_state(3);
    // test_state.setZero();
    test_state << M_PI/6, M_PI/3, 7*M_PI/4;
    Eigen::Vector2d endloc; endloc <<  manipulator.getJointLocation(test_state, 3);
    Visualizer::makeFigure(manipulator, test_state); 
    std::cout <<"joint location: " << endloc[0] << ", " << endloc[1] << "\n";
    // amp::ManipulatorState IK_state(3);
    // Eigen::Vector2d endloc; endloc << 2, 0;
    // IK_state << manipulator.getConfigurationFromIK(endloc);
    // Visualizer::makeFigure(manipulator, IK_state); 
    // std::cout<< "IK ANGLES : " << IK_state[0] << " , " << IK_state[1] << " , " << IK_state[2] << "\n";
    // std::cout <<"IK POSITION : " << IK_state[0] << ", " << endloc[1] << "\n";
    // // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    
    // Visualizer::makeFigure(manipulator, IK_state);

    // Create the collision space constructor
    std::size_t n_cells = 100;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace3());

    //You can visualize your cspace 
    Visualizer::makeFigure(*cspace);

    Visualizer::showFigures();

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "shaya.naimi@colorado.edu", argc, argv);
    return 0;
}


// amp::Polygon makeCSpaceObstacle(amp::Polygon robot, amp::Polygon obstacle){
//     //get vertices of each
//     std::vector<Eigen::Vector2d> robotpts = robot.verticesCCW();
//     std::vector<Eigen::Vector2d> obspts = obstacle.verticesCCW();
//     for (int i= 0; i<robotpts.size(); i++){ //reversing all the vertices of the ROBOT
//         robotpts[i][0] = -robotpts[i][0];
//         robotpts[i][1] = -robotpts[i][1];
//     }
//     std::vector<double> obsangles;
//     std::vector<double> robotangles;
//     std::cout<< "OBSTACLE VERTICEs " << "\n";
//     for (int i= 0; i<obspts.size(); i++){  //loop through all OBSTACLE vertices and get angles between each two

//         Eigen::Vector2d current = obspts[i];
//         Eigen::Vector2d next = obspts[0];
//         if (i < (obspts.size() - 1)){
//            next = obspts[i+1]; 
//         }
//         //get slope between current and next
//         double angle = 0;
//         std::cout << "calculating angle for " << current[0] << " , " << current[1] << "to " << next[0] << " , " << next[1] << "\n";
//         if ((next[0] - current[0]) == 0){
//             if (next[1] > current[1]){
//                 angle = M_PI / 2;
//             }
//             else{
//                 angle = 3*M_PI / 2;
//             }
//         }
//         else{
//             double slope = (next[1] - current[1])/(next[0] - current[0]);
//             std::cout << "calculated slope for " << current[0] << " , " << current[1] << "to " << next[0] << " , " << next[1] << "as " << slope << "\n";
//             if (slope == 0 & next[0] > current[0]){
//                 angle = 0;
//             }
//             else if (slope == 0 & next[0] < current[0]){
//                 angle = M_PI;
//             }
//             else if (slope > 0 & next[0] > current[0]){
//                 angle = atan(slope);
//             }
//             else if (slope > 0 & next[0] < current[0]){
//                 angle = M_PI + atan(slope);
//             }
//             else if (slope < 0 & next[0] > current[0]){
//                 angle = M_PI + atan(slope);
//             }
//             else if (slope < 0 & next[0] < current[0]){
//                 angle = atan(slope);
//             }
//             else{
//                 std::cout<< "SHOULD NEVER GET HERE; slope: " << slope << "\n";
//             }
            
//         }
//         std::cout<< "angle : " << angle << "\n";
//         obsangles.push_back(angle);
//     }
//     std::cout<< "ROBOT VERTICEs " << "\n";
//     for (int i= 0; i<robotpts.size(); i++){  //loop through all ROBOT vertices and get angles between each two

//         Eigen::Vector2d current = robotpts[i];
//         Eigen::Vector2d next = robotpts[0];
//         if (i < (robotpts.size() - 1)){
//            next = robotpts[i+1]; 
//         }
//         //get slope between current and next
//         double angle = 0;
//         std::cout << "calculating angle for " << current[0] << " , " << current[1] << "to " << next[0] << " , " << next[1] << "\n";
//         if ((next[0] - current[0]) == 0){
//             if (next[1] > current[1]){
//                 angle = M_PI / 2;
//             }
//             else{
//                 angle = 3*M_PI / 2;
//             }
//         }
//         else{
//             double slope = (next[1] - current[1])/(next[0] - current[0]);
//             //std::cout << "calculated slope for " << current[0] << " , " << current[1] << "to " << next[0] << " , " << next[1] << "as " << slope << "\n";
//             if (slope == 0 & next[0] > current[0]){
//                 angle = 0;
//             }
//             else if (slope == 0 & next[0] < current[0]){
//                 angle = M_PI;
//             }
//             else if (slope > 0 & next[0] > current[0]){
//                 angle = atan(slope);
//             }
//             else if (slope > 0 & next[0] < current[0]){
//                 angle = M_PI + atan(slope);
//             }
//             else if (slope < 0 & next[0] > current[0]){
//                 angle = M_PI + atan(slope);
//             }
//             else if (slope < 0 & next[0] < current[0]){
//                 angle = atan(slope);
//             }
//             else{
//                 std::cout<< "SHOULD NEVER GET HERE; slope: " << slope << "\n";
//             }
            
//         }
//         std::cout<< "angle : " << angle << "\n";
//         robotangles.push_back(angle);
//     }
    
//     //now, we want to put together the vertices of the cspace obstacle
//     std::vector<Eigen::Vector2d> cobspts;
//     int i = 0;
//     int j = 0;
//     obspts.push_back(obspts[0]);
//     robotpts.push_back(robotpts[0]);
//     while ((i < obspts.size()) | (j < robotpts.size())){
//         std::cout<< "HERE" << "\n";
//         Eigen::Vector2d minkovsum;
//         std::cout<< "i: " << i << ", j: " << j << "\n";
//         minkovsum <<  obspts[i][0] + robotpts[j][0], obspts[i][1] + robotpts[j][1];
//         cobspts.push_back(minkovsum);
//         if ((j == robotpts.size() - 1) && (i < obspts.size() - 1)){
//             i++;
//             continue;
//         }
//         if ((i == obspts.size() - 1) && (j < robotpts.size() - 1)){
//             std::cout<< "obspts size: " << obspts.size() << "\n";
//             j++;
//             continue;
//         }
//         if (robotangles[j] <= obsangles[i]){
//             std::cout<< "incrementing j because " << robotangles[j] << " < " << obsangles[i] << "\n";
//             j++;
//         }
//         else{
//             std::cout<< "incrementing i because " << robotangles[j] << " > " << obsangles[i] << "\n";

//             i++;
//         }
//     }
//     std::cout<< "cobs points: " << "\n";
//     for (Eigen::Vector2d vertex : cobspts){
//         std::cout << vertex[0] << " , " << vertex[1] << "\n";
//     }
//     //make a polygon out of it
//     amp::Polygon result = Polygon(cobspts);

//     return result;
 
    
// }


// amp::Polygon makeCSpaceObstacle(amp::Polygon robot, amp::Polygon obstacle){
//     //iterate through the robot points
//     std::vector<Eigen::Vector2d> robotpts = robot.verticesCCW();
//     std::vector<Eigen::Vector2d> obspts = obstacle.verticesCCW();
//     std::vector<Eigen::Vector2d> cobspts;
//     std::cout<<"listing all vertices " << "\n";
//     for (int i= 0; i<obspts.size(); i++){
//         std::cout<< obspts[i][0] << ", " << obspts[i][1] << "\n";
//     }
//     for (int i= 0; i<obspts.size(); i++){
       
//         for (int j = 0; j < robotpts.size(); j++){
//             Eigen::Vector2d vertex;
//             vertex << obspts[i][0] - robotpts[j][0], obspts[i][1] - robotpts[j][1];
//             bool dup = 0;
//             for (int k = 0; k < cobspts.size(); k++){
//                 if (cobspts[k]==vertex){
//                     dup=1;
//                 }
//             }
//             if ((dup == 0) && !((vertex[0] == 0) && (vertex[1]==0))){
//                 cobspts.push_back(vertex);
//             }
            
//         }
//     }

//     for (int i = 0; i < cobspts.size(); i++){
//         std::cout<< cobspts[i][0] << " , " << cobspts[i][1] << "\n";
//     }

//     amp::Polygon result = Polygon(cobspts);
//     return result;
// }