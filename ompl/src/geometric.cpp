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

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include "World.h"
#include "StateValidityCheckerDatabase.h"
#include "PostProcessing.h"


namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;

// this function sets-up an ompl planning problem for an arbtrary number of agents
og::SimpleSetupPtr geometricSimpleSetUp(const World *w)
{
    // grab the agent -- assume only one
    Agent *a = w->getAgents()[0];

    // create state space (with bounds)
    auto space(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);    
    bounds.setLow(0, w->getWorldDimensions()[0].first); //  x lower bound
    bounds.setHigh(0, w->getWorldDimensions()[0].second); // x upper bound
    bounds.setLow(1, w->getWorldDimensions()[1].first);  // y lower bound
    bounds.setHigh(1, w->getWorldDimensions()[1].second); // y upper bound
    space->setBounds(bounds);
 
    // define a simple setup class
    auto ss = std::make_shared<og::SimpleSetup>(space);
    
    // set state validity checker
    ss->setStateValidityChecker(std::make_shared<isStateValid_2D>(ss->getSpaceInformation(), w, a));
    
    // create start
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(a->getStartLocation()[0]);
    start->setY(a->getStartLocation()[1]);
    start->setYaw(0.0);

    // create goal
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(a->getGoalLocation()[0]);
    goal->setY(a->getGoalLocation()[1]);
    goal->setYaw(0.0);

    // save start and goal with tolerance
    ss->setStartAndGoalStates(start, goal, 0.1);

    OMPL_INFORM("Successfully Setup the problem instance");
    return ss;
}

// main planning function -- uses simple setup
void planGeometric(std::string planner_string)
{
    //create world from YAML file
    World *w = yaml2world("Problem.yml");
    // create simple setup object
    og::SimpleSetupPtr ss = geometricSimpleSetUp(w);

    // set planner
    ob::PlannerPtr planner = nullptr;
    if (planner_string == "RRT")
        planner = std::make_shared<og::RRT>(ss->getSpaceInformation());
    else if (planner_string == "SST")
        planner = std::make_shared<og::SST>(ss->getSpaceInformation());
    else
    {
        OMPL_WARN("Planner %s not found. Defaulting to RRT", planner_string.c_str());
        planner = std::make_shared<og::RRT>(ss->getSpaceInformation());
    }
    ss->setPlanner(planner);

    // run automated setup routine
    ss->setup();

    // pause for questions
    std::cout << "Setup Complete. Press ENTER to plan: ";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    // solve the instance
    bool solved = ss->solve(30.0);
    if (solved)
    {
        ss->simplifySolution();
        write2sys(ss, w->getAgents());
    }
}

int main(int argc, char ** argv)
{
    std::string plannerName = "RRT";
    OMPL_INFORM("Planning for OMPL Lecture Example using Gemoetric Planning with %s", plannerName.c_str());
    planGeometric(plannerName);
}
