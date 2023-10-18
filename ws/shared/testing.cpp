// #include "WaveFront.h"
// #include <Eigen/Dense>
// #include "HelpfulClass.h"

// struct MyNode {
//     std::pair<int, int> ind;
//     MyNode* parent;
// };

// MyNode MyPointWFAlgo::extendWave(MyNode& cell) {
//     ...
//     cout << " Searching neighbors of cell (" << a << ", " << b << ")\n";
//     for (const pair<int, int>& n : neighbors ) {
//         i = n.first;
//         j = n.second;
//         if (i >= 0 && i < gridSize.first && j >= 0 && j < gridSize.second) {
//             if (waveGrid[i][j] == 0) {
//                 ...
//                 MyNode newCell = {{i, j}, &cell};
//                 if (i == initCell.first && j == initCell.second) return newCell;
//             }
//         }
//     }
// }

// amp::Path2D MyPointWFAlgo::findPath(const MyNode& finalCell) {
//     cout << "\nRetracing path\n";
//     amp::Path2D path;
//     MyNode currCell = finalCell;
//     ...
//     while (waveGrid[i][j] != 2) {
//         i = currCell.ind.first;
//         j = currCell.ind.second;
//         cout << "Adding cell (" << i << ", " << j << ") to path\n";
//         path.waypoints.push_back({i, j});
//         currCell = *currCell.parent;
//     }
//     return path;
// }