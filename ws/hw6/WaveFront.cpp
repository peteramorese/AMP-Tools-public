// #include "CSpaceConstructor.h"
#include "WaveFront.h"
#include <Eigen/Dense>
#include "HelpfulClass.h"

using namespace amp;
using std::vector, std::string, std::cout, Eigen::Vector2d, std::pair, std::size_t;

amp::Path2D MyPointWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) { 
    initCell = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    goalCell = grid_cspace.getCellFromPoint(q_goal(0), q_goal(1)); 
    cout << "Goal cell: " << goalCell.first << ", " << goalCell.second << "\n";
    cout << "Init cell: " << initCell.first << ", " << initCell.second << "\n";
    DenseArray2D<MyNode> waveGrid(gridSize.first, gridSize.second);
    initReached = false;
    MyNode firstCell;
    firstCell.ind = goalCell;
    firstCell.value = 2;
    cellQueue = {firstCell};
    defineObstacles(grid_cspace, waveGrid);
    waveGrid(goalCell.first, goalCell.second) = firstCell;
    int step = 0;
    MyNode cell;
    while (!initReached) {
        // cell = cellQueue.back();
        // cellQueue.pop_back();
        cell = cellQueue.front();
        cellQueue.erase(cellQueue.begin());
        extendWave(cell, waveGrid);
        if (step > 10000) break;
        step++;
    }
    MyNode finalCell = cellQueue.back();
    cout << "Break at " << step << " steps\n";
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    findPath(finalCell, waveGrid, path);
    path.waypoints.push_back(q_goal);
    return path;
}

void MyPointWFAlgo::extendWave(const MyNode& cell, DenseArray2D<MyNode>& waveGrid) {
    int a = cell.ind.first;
    int b = cell.ind.second;
    int currValue = cell.value;
    // cout << " Searching neighbors of cell (" << a << ", " << b << ") with value " << currValue << " \n";
    vector<pair<int, int>> neighbors = {{a - 1, b}, {a + 1, b}, {a, b - 1}, {a, b + 1}};
    int i, j;
    for (const pair<int, int>& n : neighbors) {
        i = n.first;
        j = n.second;
        if (i >= 0 && i < gridSize.first && j >= 0 && j < gridSize.second) {
            if (waveGrid(i, j).value == 0) {
                MyNode newCell = {currValue + 1, {i, j}, cell.ind};
                waveGrid(i, j) = newCell;
                // cout << " Adding cell (" << i << ", " << j << ")\n";
                cellQueue.push_back(newCell);
                if (i == initCell.first && j == initCell.second) {
                    initReached = true;
                    break;
                }
            }
        }
    }
}

void MyPointWFAlgo::findPath(const MyNode& finalCell, const DenseArray2D<MyNode>& waveGrid, Path2D& path) {
    cout << "\nRetracing path\n";
    MyNode currCell = finalCell;
    auto[i, j] = currCell.ind;
    int ind = 0;
    while (waveGrid(i, j).value != 2) {
        // cout << "Adding cell (" << i << ", " << j << ") to path\n";
        path.waypoints.push_back(getPointFromCell({i, j}));
        auto[a, b] = waveGrid(i, j).parent;
        currCell = waveGrid(a, b);
        i = currCell.ind.first;
        j = currCell.ind.second;
        ind++;
        if ( ind == 5000) break;
    }
    path.waypoints.push_back(getPointFromCell({i, j}));
    cout << "Path length: " << ind << "\n";
}

void MyPointWFAlgo::defineObstacles(const amp::GridCSpace2D& grid_cspace, DenseArray2D<MyNode>& waveGrid) {
    int x0 = gridSize.first;
    int x1 = gridSize.second;
    for (int i = 0; i < x0; ++i) {
        for (int j = 0; j < x1; ++j){
            vector<pair<int, int>> neighbors = {{i - 1, j}, {i + 1, j}, {i, j - 1}, {i, j + 1}};
            for (const pair<int, int>& n : neighbors ) {
                if (n.first >= 0 && n.first < x0 && n.second >= 0 && n.second < x1) {
                    if (grid_cspace(n.first, n.second)) {
                        waveGrid(i, j).value = 1;
                        break;
                    } else {
                        waveGrid(i, j).value = 0;
                    }
                }
            }
        }
    }
}

Vector2d MyPointWFAlgo::getPointFromCell(const std::pair<int, int>& cell) {
    int cells0 = gridSize.first;
    int cells1 = gridSize.second;
    int i = cell.first;
    int j = cell.second;
    double x = x0lim.first + (i + 0.5 ) * (x0lim.second - x0lim.first) / cells0;
    double y = x1lim.first + (j + 0.5 ) * (x1lim.second - x1lim.first) / cells1;
    return {x, y};
}

amp::Path2D MyManipWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    initCell = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    goalCell = grid_cspace.getCellFromPoint(q_goal(0), q_goal(1)); 
    cout << "Goal cell: " << goalCell.first << ", " << goalCell.second << "\n";
    cout << "Init cell: " << initCell.first << ", " << initCell.second << "\n";
    cout << "Grid size: " << gridSize.first << ", " << gridSize.second << "\n";
    DenseArray2D<MyNode> waveGrid(gridSize.first, gridSize.second);
    initReached = false;
    MyNode firstCell;
    firstCell.ind = goalCell;
    // firstCell.parent = goalCell;
    defineObstacles(grid_cspace, waveGrid);
    firstCell.value = 2;
    cellQueue = {firstCell};
    waveGrid(goalCell.first, goalCell.second) = firstCell;
    int step = 0;
    MyNode cell;
    while (!initReached) {
        cell = cellQueue.front();
        cellQueue.erase(cellQueue.begin());
        extendWave(cell, waveGrid);
        if (step > 30000) break;
        step++;
    }
    MyNode finalCell = cellQueue.back();
    cout << "Break at " << step << " steps\n";
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    findPath(finalCell, waveGrid, path);
    path.waypoints.push_back(q_goal);
    return path;
}

void MyManipWFAlgo::defineObstacles(const amp::GridCSpace2D& grid_cspace, DenseArray2D<MyNode>& waveGrid) {
    cout << "Copying Cspace\n";
    int x0 = gridSize.first;
    int x1 = gridSize.second;
    for (int i = 0; i < x0; ++i) {
        for (int j = 0; j < x1; ++j){
            vector<pair<int, int>> neighbors = {{i - 1, j}, {i + 1, j}, {i, j - 1}, {i, j + 1}};
            for (const pair<int, int>& n : neighbors ) {
                if (n.first >= 0 && n.first < x0 && n.second >= 0 && n.second < x1) {
                    if (grid_cspace(n.first, n.second)) {
                        waveGrid(i, j).value = 1;
                        break;
                    } else {
                        waveGrid(i, j).value = 0;
                    }
                }
            }
        }
    }
}

void MyManipWFAlgo::extendWave(const MyNode& cell, DenseArray2D<MyNode>& waveGrid) {
    int a = cell.ind.first;
    int b = cell.ind.second;
    int currValue = cell.value;
    // cout << " Searching neighbors of cell (" << a << ", " << b << ") with value " << currValue << " \n";
    vector<pair<int, int>> neighbors = {{a - 1, b}, {a + 1, b}, {a, b - 1}, {a, b + 1}};
    int i, j;
    for (const pair<int, int>& n : neighbors ) {
        i = n.first;
        j = n.second;
        if (i < 0) i = gridSize.first - 1;
        if (i >= gridSize.first) i = 0;
        if (j < 0) j = gridSize.second - 1;
        if (j >= gridSize.first) j = 0;
        // cout << "Checking Cell (" << i << ", " << j << ") with value " << currValue << " \n";
        if (waveGrid(i, j).value == 0) {
            MyNode newCell = {currValue + 1, {i, j}, cell.ind};
            waveGrid(i, j) = newCell;
            // cout << " Adding cell (" << i << ", " << j << ")\n";
            cellQueue.push_back(newCell);
            if (i == initCell.first && j == initCell.second) {
                initReached = true;
                break;
            }
        }
    }
}

void MyManipWFAlgo::findPath(const MyNode& finalCell, const DenseArray2D<MyNode>& waveGrid, amp::Path2D& path) {
    cout << "\nRetracing path\n";
    MyNode currCell = finalCell;
    auto[i, j] = currCell.ind;
    int ind = 0;
    while (waveGrid(i, j).value != 2) {
        // cout << "Adding cell (" << i << ", " << j << ") to path\n";
        path.waypoints.push_back(getPointFromCell({i, j}));
        auto[a, b] = waveGrid(i, j).parent;
        currCell = waveGrid(a, b);
        i = currCell.ind.first;
        j = currCell.ind.second;
        ind++;
        if ( ind == 5000) break;
    }
    path.waypoints.push_back(getPointFromCell({i, j}));
    cout << "Path length: " << ind << "\n";
}

Vector2d MyManipWFAlgo::getPointFromCell(const std::pair<int, int>& cell) {
    int cells0 = gridSize.first;
    int cells1 = gridSize.second;
    int i = cell.first;
    int j = cell.second;
    double x = x0lim.first + (i + 0.5 ) * (x0lim.second - x0lim.first) / cells0;
    double y = x1lim.first + (j + 0.5 ) * (x1lim.second - x1lim.first) / cells1;
    return {x, y};
}