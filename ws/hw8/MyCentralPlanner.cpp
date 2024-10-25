#include "MyMultiAgentPlanners.h"
#include "GenericPRM.h"
#include "myMACollChecker.h"


amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    int numAgens = problem.numAgents();
    std::cout << "numAgents: " << numAgens << std::endl;
    amp::GenericPRM generic_planner;
    generic_planner.set_problem(problem);
    Eigen::VectorXd lower = Eigen::Vector2d{problem.x_min, problem.y_min};
    Eigen::VectorXd upper = Eigen::Vector2d{problem.x_max, problem.y_max};
    MyMACollChecker cspace = MyMACollChecker(lower, upper);
    Eigen::VectorXd all_q_init;
    Eigen::VectorXd all_q_goal;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        all_q_init.conservativeResize(all_q_init.size() + 2);
        all_q_init.segment(all_q_init.size() - 2, 2) = agent.q_init;
        all_q_goal.conservativeResize(all_q_goal.size() + 2);
        all_q_goal.segment(all_q_goal.size() - 2, 2) = agent.q_goal;
    }
    amp::Path path_nd = generic_planner.plan(all_q_init, all_q_goal, cspace);

    //create an Xd vector (dimension = 2m)of their initial states, and one of their final states
    //use RRT to find a path from the initial state to the final state
    //will need to generalize RRT to work with nD vectors
    //also will need to create a new collision checker that checks for agent-obstacle collision but also agent-agent collision
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    return path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    return path;
}