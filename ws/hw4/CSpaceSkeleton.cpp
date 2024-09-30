#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    int ncells = 100;
    double xlower = 0;
    double xupper = 2*M_PI;
    double cellsize = (xupper-xlower) / ncells;
    int cellx = (x0-xlower)/cellsize;
    int celly = (x1-xlower)/cellsize;

    std::size_t cell_x = cellx;
    std::size_t cell_y = celly;


    //get x cell
    //first, get cell size
    //divide x0 by cell size
    //convert to an integer

    //std::size_t cell_x = 0; // x index of cell
    //std::size_t cell_y = 0; // x index of cells
    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    int ncells = 100;
    double xlower = 0;
    double xupper = 2*M_PI;
    double ylower = 0;
    double yupper = 2*M_PI;
    double cellsize = (xupper-xlower) / ncells;
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(ncells, ncells, xlower, xupper, ylower, yupper);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    

    //for loop: for each cell in the grid
    for (int i = 0; i < ncells; i++){
        for (int j = 0; j < ncells; j++){
            double samplex = (i * cellsize) + xlower + cellsize/2;
            double sampley = (j * cellsize) + xlower + cellsize/2;

            
            //do FORWARD KINEMATICS TO GET X AND Y
            
            //std::cout << "testing2" << "\n";
            amp::ManipulatorState joint_angles(3);
            joint_angles << samplex, sampley;
            

            //CHECK OBSTACLE COLLISION
            for (int n = 0; n <= manipulator.nLinks(); n++){
                Eigen::Vector2d startpoint; startpoint = manipulator.getJointLocation(joint_angles, n);
                Eigen::Vector2d endpoint; endpoint = manipulator.getJointLocation(joint_angles, n+1);
                
                for (double t = startpoint[0]; t <= endpoint[0]; t+=.05 ){
                    for (amp::Obstacle2D obs : env.obstacles){
                    std::vector<Eigen::Vector2d> points = obs.verticesCCW();
                    int k, m, nvert = points.size();
                    bool c = false;
                    Eigen::Vector2d point; point << startpoint[0] + t, startpoint[1] + t*((endpoint[1]-startpoint[1])/(endpoint[0]-startpoint[0]));
                    for(k = 0, m = nvert - 1; k < nvert; m = k++) {
                        if( ( (points[k][1] >= point[1] ) != (points[m][1] >= point[1]) ) &&
                            (point[0] <= (points[m][0] - points[k][0]) * (point[1] - points[k][1]) / (points[m][1] - points[k][1]) + points[k][0])
                        )   
                        c = !c;
                        }
                        if (c){
                            cspace(i, j) = true;
                        }
                    }
                
                }

            }
                

        }
    }

    //now, go through and check all the cells, and if they're surrounded on at least two neighbors by obstacle, then it's obs
    for (int i = 1; i < ncells-1; i++){
        for (int j = 1; j < ncells-1; j++){
            std::vector<Eigen::Vector2d> neighbors;
            neighbors.push_back(Eigen::Vector2d(i-1, j));
            neighbors.push_back(Eigen::Vector2d(i+1, j));
            neighbors.push_back(Eigen::Vector2d(i, j+1));
            neighbors.push_back(Eigen::Vector2d(i, j-1));
            int neighbcount = 0;
            for (int k = 0; k < 4; k++){
                if (cspace(neighbors[k][0], neighbors[k][1]) == true){
                    neighbcount++;
                }
            }
            if (neighbcount > 2){
                cspace(i, j) = true;
            }
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

