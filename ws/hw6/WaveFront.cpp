// #include "CSpaceConstructor.h"
#include "WaveFront.h"
#include <Eigen/Dense>
#include "HelpfulClass.h"

using namespace amp;
using std::vector, std::string, std::cout, Eigen::Vector2d, std::pair, std::size_t;

amp::Path2D MyPointWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) { 
    initCell = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    goalCell = grid_cspace.getCellFromPoint(q_goal(0), q_goal(1));
    initReached = false;
    Cell firstCell;
    cellQueue = {goalCell};
    Cell cell;
    grid_cspace(goalCell.first, goalCell.second) = 2;
    int step = 0;
    while (!initReached) {
        cell = cellQueue.back();
        cellQueue.pop_back();
        extendWave(cell, grid_cspace);
        if (step > 1000) break;
        step++;
    }
    Cell finalCell = cellQueue.back();

    cout << "Break at " << step << " steps\n";
    return findPath(finalCell);
}

void MyPointWFAlgo::extendWave(const Cell& cell, const amp::GridCSpace2D& grid_cspace) {
    auto[a, b] = cell;
    int currValue = grid_cspace(a, b);
    vector<pair<int, int>> neighbors = {{a - 1, b}, {a + 1, b}, {a, b - 1}, {a, b + 1}};
    int i, j;
    for (const pair<int, int>& n : neighbors ) {
        i = n.first;
        j = n.second;
        if (grid_cspace(i, j) == 0) {
            grid_cspace(i, j) = currValue + 1;
            Cell newCell = {{i, j}, &cell};
            cellQueue.push_back(newCell);
            if (i == initCell.first && j == initCell.second) initReached = true;
        }
    }
}

amp::Path2D MyPointWFAlgo::findPath(const Cell& finalCell) {
    amp::Path2D path;
    Cell currCell = finalCell;
    int i, j;
    [i, j] = currCell.cell;
    while (grid_cspace(i, j) != 2) {
        path.waypoints.push_back({i, j});
        currCell = currCell.parent;
        [i, j] = currCell.cell;
    }
}