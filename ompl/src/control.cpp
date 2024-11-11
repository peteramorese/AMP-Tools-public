/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Justin Kottinger */

#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include "World.h"
#include "StateSpaceDatabase.h"
#include "ControlSpaceDatabase.h"
#include "StateValidityCheckerDatabase.h"
#include "StatePropagatorDatabase.h"
#include "GoalRegionDatabase.h"
#include "PostProcessing.h"


namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace oc = ompl::control;

// this function sets-up an ompl planning problem for an arbtrary number of agents
oc::SimpleSetupPtr controlSimpleSetUp(const World *w)
{
    // Grab the agent -- assume only one for demo purposes
    Agent *a = w->getAgents()[0];

    // Create state and control spaces
    ob::StateSpacePtr space = createBounded2ndOrderCarStateSpace(w->getWorldDimensions()[0], w->getWorldDimensions()[1]);
    oc::ControlSpacePtr cspace = createUniform2DRealVectorControlSpace(space);

    // Define a simple setup class from the state and control spaces
    auto ss = std::make_shared<oc::SimpleSetup>(cspace);

    // Set state validity checker (includes collision checking)
    ss->setStateValidityChecker(std::make_shared<isStateValid_2D>(ss->getSpaceInformation(), w, a));

    // Use the ODESolver to propagate the system. Call SecondOrderCarODE to intgrate dynamics and 
    // call SecondOrderCarODEPostIntegration to post process (normalize orientation)
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &SecondOrderCarODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &SecondOrderCarODEPostIntegration));

    // Create start state
    ob::ScopedState<> start(space);
    start[0] = a->getStartLocation()[0];
    start[1] = a->getStartLocation()[1];
    start[2] = 0;
    start[3] = 0;
    start[4] = 0;

    // Create goal region (defined in GoalRegionDatabase.h)
    ob::GoalPtr goal(new GoalRegion2ndOrderCar(ss->getSpaceInformation(), a->getGoalLocation()[0], a->getGoalLocation()[1]));
    
    // Set propagation step size and min/max number of steps
    // ss->getSpaceInformation()->setPropagationStepSize(0.1);
    // ss->getSpaceInformation()->setMinMaxControlDuration(10, 20);
    
    // Add start and goal to problem setup
    ss->setStartState(start);
    ss->setGoal(goal);

    OMPL_INFORM("Successfully Setup the problem instance");
    return ss;
}

// main planning function -- uses simple setup
void planControl(std::string planner_string, std::string problem_file) {
    // Create world from YAML file
    World *w = yaml2world("problems/" + problem_file + ".yml");
    // Create simple setup object
    oc::SimpleSetupPtr ss = controlSimpleSetUp(w);

    // Select planner
    ob::PlannerPtr planner = nullptr;
    if (planner_string == "RRT")
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
    else if (planner_string == "SST")
        planner = std::make_shared<oc::SST>(ss->getSpaceInformation());
    else {
        OMPL_WARN("Planner %s not found. Defaulting to RRT", planner_string.c_str());
        planner = std::make_shared<oc::RRT>(ss->getSpaceInformation());
    }
    ss->setPlanner(planner);
    
    // run automated setup routine
    ss->setup();

    // pause for questions
    std::cout << "Setup Complete. Press ENTER to plan: ";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    // solve the instance with a 30 second timeout
    double max_time = 30.0;
    bool solved = ss->solve(max_time);
    if (solved)
        write2sys(ss, w->getAgents(), problem_file);
}

int main(int argc, char ** argv) {
    std::string plannerName = "RRT";
    // std::string problem = "DemoCntrl";
    std::string problem = "WS1Car";
    OMPL_INFORM("Planning for OMPL Lecture Example using Control Planning with %s", plannerName.c_str());
    planControl(plannerName, problem);
}
