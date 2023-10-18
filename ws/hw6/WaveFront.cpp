// #include "CSpaceConstructor.h"
#include "WaveFront.h"
#include <Eigen/Dense>
#include "HelpfulClass.h"

using namespace amp;
using std::vector, std::string, std::cout, Eigen::Vector2d, std::pair, std::size_t;

amp::Path2D MyPointWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) { 
    initCell = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    goalCell = grid_cspace.getCellFromPoint(q_goal(0), q_goal(1)); 
    cout << goalCell.first << ", " << goalCell.second << "\n";
    cout << initCell.first << ", " << initCell.second << "\n";
    initReached = false;
    MyNode firstCell;
    firstCell.ind = goalCell;
    firstCell.parent = &firstCell;
    cellQueue = {firstCell};
    defineObstacles(grid_cspace);
    waveGrid[goalCell.first][goalCell.second] = 2;
    int step = 0;
    MyNode cell;
    while (!initReached) {
        cell = cellQueue.back();
        cellQueue.pop_back();
        extendWave(cell);
        if (step > 5000) break;
        step++;
    }
    MyNode finalCell = cellQueue.back();
    cout << "Break at " << step << " steps\n";
    return findPath(finalCell);
}

void MyPointWFAlgo::extendWave(MyNode& cell) {
    int a = cell.ind.first;
    int b = cell.ind.second;
    int currValue = waveGrid[a][b];
    // cout << " Searching neighbors of cell (" << a << ", " << b << ") with value " << currValue << " \n";
    vector<pair<int, int>> neighbors = {{a - 1, b}, {a + 1, b}, {a, b - 1}, {a, b + 1}};
    int i, j;
    for (const pair<int, int>& n : neighbors ) {
        i = n.first;
        j = n.second;
        if (i >= 0 && i < gridSize.first && j >= 0 && j < gridSize.second) {
            if (waveGrid[i][j] == 0) {
                waveGrid[i][j] = currValue + 1;
                // cout << " Adding cell (" << i << ", " << j << ")\n";
                MyNode newCell = {{i, j}, &cell};
                cellQueue.push_back(newCell);
                if (i == initCell.first && j == initCell.second) {
                    initReached = true;
                    break;
                }
            }
        }
    }
}

amp::Path2D MyPointWFAlgo::findPath(const MyNode& finalCell) {
    cout << "\nRetracing path\n";
    amp::Path2D path;
    MyNode currCell = finalCell;
    int i = currCell.ind.first;
    int j = currCell.ind.second;
    int ind = 0;
    while (waveGrid[i][j] != 2) {
        i = currCell.ind.first;
        j = currCell.ind.second;
        cout << "Adding cell (" << i << ", " << j << ") to path\n";
        path.waypoints.push_back({i, j});
        currCell = *currCell.parent;
        ind++;
        if ( ind == 5) break;
    }
    return path;
}

void MyPointWFAlgo::defineObstacles(const amp::GridCSpace2D& grid_cspace) {
    std::pair<int, int> gridSize = grid_cspace.size();
    int x0 = gridSize.first;
    int x1 = gridSize.second;
    for (int i = 0; i < x0; ++i) {
        for (int j = 0; j < x1; ++j){
            waveGrid[i][j] = grid_cspace.operator()(i, j);
            // cout << grid_cspace.operator()(i, j) << " mapped to waveGrid at " << i << ", " << j << "\n";
        }
    }
}
