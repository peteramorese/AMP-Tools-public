#include "MyMultiAgentPlanners.h"
#include "GenericPRM.h"
#include "myMACollChecker.h"
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"


amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    int numAgens = problem.numAgents();
    std::cout << "numAgents: " << numAgens << std::endl;
    double agent_radius = problem.agent_properties[0].radius;
    std::vector<double> agent_radii(numAgens, agent_radius);
    for (int i = 1; i < numAgens; ++i) {
        agent_radii[i] = problem.agent_properties[i].radius;
    }
    // std::cout << "agent radii: " << agent_radii << std::endl;

    amp::GenericPRM generic_planner;
    generic_planner.set_problem(problem);
    generic_planner.myN = myN;
    generic_planner.myr = myr;
    // std::cout << "xmin: " << problem.x_min << " xmax: " << problem.x_max << "\n";
    // std::cout << " ymin: " << problem.y_min <<  " ymax: " << problem.y_max << std::endl;
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
    cspace.set_robot_radius(*std::min_element(agent_radii.begin(), agent_radii.end())*2);
    cspace.set_agent_radii(agent_radii);
    Eigen::VectorXd all_q_init;
    Eigen::VectorXd all_q_goal;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        all_q_init.conservativeResize(all_q_init.size() + 2);
        all_q_init.segment(all_q_init.size() - 2, 2) = agent.q_init;
        all_q_goal.conservativeResize(all_q_goal.size() + 2);
        all_q_goal.segment(all_q_goal.size() - 2, 2) = agent.q_goal;
    }
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
     for (const amp::CircularAgentProperties& agent : problem.agent_properties) { //for each agent
        amp::Path2D agent_path; 
        // agent_path.waypoints = {agent.q_init};
        path.agent_paths.push_back(agent_path);
    }
  
    amp::Path path_nd = generic_planner.plan(all_q_init, all_q_goal, cspace);
    treeSize = generic_planner.treeSize;
    
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
    //GENERAL SETUP
    amp::MultiAgentPath2D path;
    int numAgens = problem.numAgents();
    std::cout << "numAgents: " << numAgens << std::endl;
    double agent_radius = problem.agent_properties[0].radius;
    std::vector<double> agent_radii(numAgens, agent_radius);
    for (int i = 1; i < numAgens; ++i) {
        agent_radii[i] = problem.agent_properties[i].radius;
    }
    // std::cout << "agent_radius: " << agent_radius << std::endl;
    amp::MyRRTDecent generic_planner;
    generic_planner.set_problem(problem);
    generic_planner.myN = myN;
    generic_planner.myr = myr;
    // std::cout << "xmin: " << problem.x_min << " xmax: " << problem.x_max << "\n";
    // std::cout << " ymin: " << problem.y_min <<  " ymax: " << problem.y_max << std::endl;
    // Eigen::VectorXd lower(2 * numAgens);
    // for (int i = 0; i < numAgens; ++i) {
    //     lower[2 * i] = problem.x_min;
    //     lower[2 * i + 1] = problem.y_min;
    // }
    // Eigen::VectorXd upper(2 * numAgens);
    // for (int i = 0; i < numAgens; ++i) {
    //     upper[2 * i] = problem.x_max;
    //     upper[2 * i + 1] = problem.y_max;
    // }
    Eigen::VectorXd lower = Eigen::Vector2d{problem.x_min, problem.y_min};
    Eigen::VectorXd upper = Eigen::Vector2d{problem.x_max, problem.y_max};
    MyMACollChecker cspace = MyMACollChecker(lower, upper);
    cspace.set_robot_radius(*std::min_element(agent_radii.begin(), agent_radii.end())*2);
    cspace.set_agent_radii(agent_radii);

    //DECENTRALIZED PLAN
    int totalsteps = 0; //keep a number of steps equal to the total number of timesteps so far in the problem
    //for each agent:
    for (int i = 0; i < numAgens; ++i) {
        std::cout << "started planning for agent " << i << std::endl;
        amp::Path2D agent_path;
        for (int j = 0; j < totalsteps; ++j) {
            agent_path.waypoints.push_back(problem.agent_properties[i].q_init);
        }
        // std::cout << "added initial state to path for agent " << i << std::endl;
        amp::Path2D moving_path = generic_planner.plan(i, problem, cspace);  
        // std::cout << "got plan for agent " << i << std::endl;
        agent_path.waypoints.insert(agent_path.waypoints.end(), moving_path.waypoints.begin(), moving_path.waypoints.end());
        path.agent_paths.push_back(agent_path);
        totalsteps += moving_path.waypoints.size();
        // std::cout << "finished planning for agent " << i << std::endl;
     }

    // Add the final state to each agent's path until it reaches totalsteps
    for (int i = 0; i < numAgens; ++i) {
        while (path.agent_paths[i].waypoints.size() < totalsteps) {
            path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal);
        }
    }


    //first, add their initial state to the path totalsteps times
    // then, plan a path from their initial state to their final state and update totalsteps

    //after looping through all agents: go back to each agent and add their final state to the path until it gets to totalsteps

    // for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
    //     amp::Path2D agent_path;
    //     agent_path.waypoints = {agent.q_init, agent.q_goal};
    //     path.agent_paths.push_back(agent_path);
    // }
    return path;
}