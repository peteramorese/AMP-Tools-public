// #include "MyBugAlgorithm.h"

// // Implement your methods in the `.cpp` file, for example:
// amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D &problem) 
// {
//     cout << "Starting Bug " << bugType << "\n";
//     x = problem.q_init(0);
//     y = problem.q_init(1);
//     start = {x, y};
//     positionHistory.push_back(start);
//     goal = {problem.q_goal(0), problem.q_goal(1)};
//     edges = findEdges(problem);
//     mLine = findLineEquation(start, goal);
//     shortestPath = distanceBetweenPoints(start, goal);
//     turnToGoal();
//     for (int i = 0; i < 2000; ++i)
//     {
//         moveForward();
//         detectEdges();
//         if (bugType == 1)
//         {
//             detectEntryExitPoints();
//         }
//         else
//         {
//             detectMLine();
//         }
//         if (detectPoint(goal))
//         {
//             cout << "Goal reached in " << i << " steps\n";
//             break;
//         }
//     }
//     return path;
// }
