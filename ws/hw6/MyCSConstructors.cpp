#include "MyCSConstructors.h"
#include "hw/HW4.h"

Eigen::Vector2d gJL(const amp::ManipulatorState& state, uint32_t joint_index, std::vector<double> link_lengths)  {
    std::cout << "HERE" << "\n";
    Eigen::Vector2d result; result << 0, 0;
    double theta = 0;
    for (int i = 0; i < joint_index; i++){
        theta+= state[i];
        result[0] += link_lengths[i]*cos(theta);
        result[1] += link_lengths[i]*sin(theta);
    }
    return result;
}

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    double cellsize = 0.25;
    std::pair<double, double> x0bounds = x0Bounds();
    std::pair<double, double> x1bounds = x1Bounds();
    // std::pair<double, double> x0_cells, x1_cells = size();
    //double x0_cellsize = (x0_max-x0_min) / x0_cells;
    //double x1_cellsize = (x1_max-x1_min) / x1_cells;
    double x0_min = x0bounds.first;
    int cellx = (x0-x0bounds.first)/cellsize;
    int celly = (x1-x1bounds.first)/cellsize;

    std::size_t cell_x = cellx;
    std::size_t cell_y = celly;
    return {cell_x, cell_y};
    //return {0 , 0};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
// std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
//     // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
//     // Pass the constructor parameters to std::make_unique()
//     size_t m_cells = (env.x_max - env.x_min) / 0.25;
//     size_t n_cells = (env.y_max - env.y_min) / 0.25;

//     std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells, n_cells, env.x_min, env.x_max, env.y_min, env.y_max);
//     // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
//     MyGridCSpace2D& cspace = *cspace_ptr;
//     std::cout << "Constructing C-space for manipulator" << std::endl;
//     // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
//     cspace(1, 3) = true;
//     cspace(3, 3) = true;
//     cspace(0, 1) = true;
//     cspace(1, 0) = true;
//     cspace(2, 0) = true;
//     cspace(3, 0) = true;
//     cspace(4, 1) = true;

//     // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
//     // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
//     return cspace_ptr;
// }

std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    int ncells = 2*M_PI/0.25;
    //ncells = 100;
    double xlower = 0;
    double xupper = 2*M_PI;
    double ylower = 0;
    double yupper = 2*M_PI;
    double cellsize = 0.25;
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(ncells, ncells, xlower, xupper, ylower, yupper);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    

    //for loop: for each cell in the grid
    for (int i = 0; i < ncells; i++){
        for (int j = 0; j < ncells; j++){
            std::cout << "checking cell: " << i << ", " << j << "\n";
            double samplex = (i * cellsize) + xlower + cellsize/2;
            double sampley = (j * cellsize) + xlower + cellsize/2;

            
            //do FORWARD KINEMATICS TO GET X AND Y
            
            //std::cout << "testing2" << "\n";
            amp::ManipulatorState joint_angles(3);
            joint_angles << samplex, sampley;
            std::cout << "joint_angles: " << joint_angles << "\n";
            

            //CHECK OBSTACLE COLLISION
            for (int n = 0; n <= manipulator.nLinks(); n++){
                std::vector<double> linklengths = manipulator.getLinkLengths();
                Eigen::Vector2d startpoint; startpoint = gJL(joint_angles, n, linklengths);
                Eigen::Vector2d endpoint; endpoint = gJL(joint_angles, n+1, linklengths) ;
                std::cout << "startpoint: " << startpoint << " endpoint: " << endpoint << "\n";
                
                for (double t = startpoint[0]; t <= endpoint[0]; t+=.1 ){
                    std::cout<<"stuck in loop" << "\n";
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
    // for (int i = 1; i < ncells-1; i++){
    //     for (int j = 1; j < ncells-1; j++){
    //         std::vector<Eigen::Vector2d> neighbors;
    //         neighbors.push_back(Eigen::Vector2d(i-1, j));
    //         neighbors.push_back(Eigen::Vector2d(i+1, j));
    //         neighbors.push_back(Eigen::Vector2d(i, j+1));
    //         neighbors.push_back(Eigen::Vector2d(i, j-1));
    //         int neighbcount = 0;
    //         for (int k = 0; k < 4; k++){
    //             if (cspace(neighbors[k][0], neighbors[k][1]) == true){
    //                 neighbcount++;
    //             }
    //         }
    //         if (neighbcount > 2){
    //             cspace(i, j) = true;
    //         }
    //     }
    //}

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

//////////////////////////////////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {

    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    size_t m_cells = (env.x_max - env.x_min) / 0.25;
    size_t n_cells = (env.y_max - env.y_min) / 0.25;
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells, n_cells, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    double x_cell_size = (env.x_max - env.x_min) / m_cells;
    double y_cell_size = (env.y_max - env.y_min) / n_cells;
    for (int i = 0; i < m_cells; i++){
        for (int j = 0; j < n_cells; j++){
            for (amp::Obstacle2D obs : env.obstacles){
                    
                    std::vector<Eigen::Vector2d> points = obs.verticesCCW();
                    Eigen::Vector2d point = Eigen::Vector2d((i * x_cell_size) + env.x_min + x_cell_size/2, (j * y_cell_size) + env.y_min + y_cell_size/2);
                    int k, m, nvert = points.size();
                    bool c = false;
                    for(k = 0, m = nvert - 1; k < nvert; m = k++) {
                        if( ( (points[k][1] >= point[1] ) != (points[m][1] >= point[1]) ) &&
                            (point[0] <= (points[m][0] - points[k][0]) * (point[1] - points[k][1]) / (points[m][1] - points[k][1]) + points[k][0])
                        )   
                        c = !c;
                        }
                        if (c){
                            //std::cout<<"true for " << i << "," << j << "\n";
                            cspace(i, j) = true;
                        }
                    }

        }
    }

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
   return cspace_ptr;
}

std::vector<std::pair<std::size_t, std::size_t>> MyGridCSpace2D::getGridNeighbors(int i, int j) const{
std::vector<std::pair<std::size_t, std::size_t>> neighbors;
std::pair<std::size_t, std::size_t> grid_dim = size();
    for (int x = -1; x <= 1; x++){
        for (int y = -1; y <= 1; y++){
            if (x == 0 && y == 0) continue;
            std::size_t n_i = i + x;
            std::size_t n_j = j + y;
            if (n_i < grid_dim.first && n_j < grid_dim.second)
                neighbors.push_back({n_i, n_j});
        }
    }
    return neighbors;
    
}

<<<<<<< HEAD
amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    //first, create a 2d array with the same number of cells as gridspace; set all values to0
    // then, find the goal and set it to 2 or something
    //loop through all the cells in the cspace
    std::pair<size_t, size_t> grid_dim = grid_cspace.size();
    std::cout << "grid cell dim: " << grid_dim.first << ", " << grid_dim.second << "\n";
    double cell_size = (grid_cspace.x0Bounds().second - grid_cspace.x0Bounds().first)/grid_cspace.size().first;
    int wave_array[grid_dim.first][grid_dim.second];
    std::pair<std::size_t, std::size_t> goal_cell = grid_cspace.getCellFromPoint(q_goal(0), q_goal(1));
    
    // std::cout << "grid dimensions 1: " << grid_cspace.x0Bounds().first << "," << grid_cspace.x0Bounds().second << "\n";
    // std::cout << "grid dimensions 2: " << grid_cspace.x1Bounds().first << "," << grid_cspace.x1Bounds().second << "\n";

    for (size_t i = 0; i < grid_dim.first; i++){
        
            for (size_t j = 0; j < grid_dim.second; j++){
                
                Eigen::Vector2d midpoint = Eigen::Vector2d(i, j);
                //TODO: write cell to point function
                // midpoint[0] = midpoint[0] * grid_dim.first * grid_cspace.x0Bounds().first; 
                midpoint[0] = grid_cspace.x0Bounds().first + i * 0.25 - 0.25/2;
                midpoint[1] = grid_cspace.x1Bounds().first + j * 0.25 - 0.25/2;
                //std::cout<< "midpoint: " << midpoint[0] << "," << midpoint[1] << "\n";
                std::pair<std::size_t, std::size_t> grid_cell = grid_cspace.getCellFromPoint(midpoint[0], midpoint[1]);
                if (grid_cspace(grid_cell.first, grid_cell.second) == 1){//if it's in a collision
                    wave_array[i][j] = 1;
                    //std::cout << "obstacle at: " << i << "," << j << "\n";
                    //std::cout << "cell: " << midpoint[0] << "," << midpoint[1] << "\n";
                }
                else{
                    wave_array[i][j] = 0;
                }
            }
            if (i == grid_dim.first - 1) {
                std::cout << "break\n";
                break;}
        }
    int level = 2;
    wave_array[goal_cell.first][goal_cell.second] = 2;
    //WHILE there are unvisited cells
    // std::cout << "Dimensions of wave_array2: " << sizeof(wave_array) << ", " << sizeof(wave_array[0]) << "\n";

    std::pair<std::size_t, std::size_t> test_cell = grid_cspace.getCellFromPoint(-4.75, 5.57);
    // std::cout << "Value of cell at (-4.75, 5.57): " << wave_array[test_cell.first][test_cell.second] << "\n";
    
//    while (true){ 
    for (int i = 0; i < grid_dim.first; i++){
    
        

        //std::cout << "wave_array: " << wave_array << "\n";       
        //std::cout<< "level: " << level  << "\n";
        size_t gdf = grid_dim.first;
        size_t gds = grid_dim.second;
        
        for (size_t i = 0; i < gdf; i++){
            for (size_t j = 0; j < gds; j++){

                if (wave_array[i][j] == 0){
                    std::vector<std::pair<std::size_t, std::size_t>> neighbors = grid_cspace.getGridNeighbors(i, j);
                    //std::cout << "number of neighbors: " << neighbors.size() << "\n";
                    for (auto neighbor : neighbors){
                        if (wave_array[neighbor.first][neighbor.second] == level){
                            //std::cout<< "assigning ij to " << i << "," << j << " " << wave_array[i][j] << "\n";
                            wave_array[i][j] = level+1;
                            break;
                        }
                    }
                }
            }
        }
        level++;
        bool unvisited_cells = false;
        for (size_t i = 0; i < gdf; i++){
            for (size_t j = 0; j < gds; j++){
                if (wave_array[i][j] == 0){
                    unvisited_cells = true;
                }
            }
        }
        if (!unvisited_cells){
            break;
        }
    }
    bool print_wave_array = false;
    if (print_wave_array){
        for (size_t i = 0; i < grid_dim.first; i++){
        for (size_t j = 0; j < grid_dim.second; j++){
            std::cout << wave_array[i][j] << " ";
        }
        std::cout << "\n";
    }
    }
    
    
    test_cell = grid_cspace.getCellFromPoint(-4.75, 5.57);
    std::cout << "Value of cell at (-4.75, 5.57): " << wave_array[test_cell.first][test_cell.second] << "\n";
=======
amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here
>>>>>>> peteramorese-main
    amp::Path2D path;
    std::cout << "Dimensions of wave_array3: " << sizeof(wave_array) << ", " << sizeof(wave_array[0]) << "\n";
    
    //now, plan from q_init to q_goal
    //get the cell value of the initial cell
    std::pair<std::size_t, std::size_t> init_cell = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    std::cout << "Initial cell: " << init_cell.first << ", " << init_cell.second << "\n";

    std::pair<std::size_t, std::size_t> curr_cell = init_cell;
    // while (curr_cell != goal_cell){
    for (int i = 0; i < 10; i++){
        std::vector<std::pair<std::size_t, std::size_t>> neighbors = grid_cspace.getGridNeighbors(curr_cell.first, curr_cell.second);
        int min_level = wave_array[curr_cell.first][curr_cell.second];
        std::pair<std::size_t, std::size_t> next_cell = curr_cell;
        //std::cout<< "min level: " << min_level << "\n";
        for (auto neighbor : neighbors){
            //std::cout<< "neighbor value:" << wave_array[neighbor.first][neighbor.second] << "\n";
           
            if (wave_array[neighbor.first][neighbor.second] == min_level - 1){
                min_level = wave_array[neighbor.first][neighbor.second];
                next_cell = neighbor;
                Eigen::Vector2d midpoint = Eigen::Vector2d(next_cell.first, next_cell.second);
                midpoint[0] = grid_cspace.x0Bounds().first + midpoint[0] * cell_size;
                midpoint[1] = grid_cspace.x1Bounds().first + midpoint[1] * cell_size;
                path.waypoints.push_back(midpoint);
                curr_cell = next_cell;
            }
            if (curr_cell == goal_cell){
                break;
            }
        }
        //std::cout << "might be stuck " << "\n";
        
    }
    
   
    std::cout<< grid_cspace.inCollision(0, 0) << " <- IN COLLIS \n"; 

    path.waypoints.push_back(q_goal);
    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}


