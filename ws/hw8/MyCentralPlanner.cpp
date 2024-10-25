#include "MyMultiAgentPlanners.h"
#include "GenericPRM.h"
#include "myMACollChecker.h"


amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    int numAgens = problem.numAgents();
    std::cout << "numAgents: " << numAgens << std::endl;
    amp::GenericPRM generic_planner;
    generic_planner.set_problem(problem);
    Eigen::VectorXd lower(2 * numAgens);
    for (int i = 0; i < numAgens; ++i) {
        lower[2 * i] = problem.x_min;
        lower[2 * i + 1] = problem.y_min;
    }
    Eigen::VectorXd upper(2 * numAgens);
    for (int i = 0; i < numAgens; ++i) {
        upper[2 * i] = problem.x_max;
        upper[2 * i + 1] = problem.y_max;
    }
    MyMACollChecker cspace = MyMACollChecker(lower, upper);
    cspace.set_robot_radius(1);
    Eigen::VectorXd all_q_init;
    Eigen::VectorXd all_q_goal;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        all_q_init.conservativeResize(all_q_init.size() + 2);
        all_q_init.segment(all_q_init.size() - 2, 2) = agent.q_init;
        all_q_goal.conservativeResize(all_q_goal.size() + 2);
        all_q_goal.segment(all_q_goal.size() - 2, 2) = agent.q_goal;
    }
    amp::Path path_nd = generic_planner.plan(all_q_init, all_q_goal, cspace);
     for (const amp::CircularAgentProperties& agent : problem.agent_properties) { //for each agent
        amp::Path2D agent_path; 
        // agent_path.waypoints = {agent.q_init};
        path.agent_paths.push_back(agent_path);
    }
    // std::cout << "number of agent paths: " << path.agent_paths.size() << std::endl;
    for (const auto& waypoint : path_nd.waypoints) { //for each waypoint
        for (int i = 0; i < numAgens; ++i) { //for each agent
            path.agent_paths[i].waypoints.push_back(waypoint.segment<2>(2 * i)); //add the waypoint
        }

    }
    return path;
    }

    //create an Xd vector (dimension = 2m)of their initial states, and one of their final states
    //use RRT to find a path from the initial state to the final state
    //will need to generalize RRT to work with nD vectors
    // //also will need to create a new collision checker that checks for agent-obstacle collision but also agent-agent collision
    // for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
    //     amp::Path2D agent_path;
    //     agent_path.waypoints = {agent.q_init, agent.q_goal};
    //     path.agent_paths.push_back(agent_path);
    // }
    // return path;

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    return path;
}