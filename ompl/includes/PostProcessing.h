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

#include <filesystem>
#include <fstream>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>

namespace fs = std::filesystem;
namespace og = ompl::geometric;
namespace oc = ompl::control;


// generate date/time information to solutions or solution directories
std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y_%m_%d_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '_');
    return s;
}

// parent function for including date/time information to files
fs::path appendTimeToFileName(const fs::path& fileName)
{
    return fileName.stem().string() + "_" + GetCurrentTimeForFileName() + fileName.extension().string();
}

// write solultion to the system
void write2sys(const og::SimpleSetupPtr problem, const std::vector<Agent*> agents)
{
    fs::path sol_dir = "solutions/" + GetCurrentTimeForFileName();
    fs::create_directories(sol_dir);

    std::string fileName = agents[0]->getName() + ".txt";
    auto filePath = fs::current_path() / sol_dir / fs::path(fileName); /// appendTimeToFileName(fileName); // e.g. MyPrettyFile_2018-06-09_01-42-00.txt
    std::ofstream file(filePath);
    const og::PathGeometric p = problem->getSolutionPath();
    p.printAsMatrix(file);
}

// write solultion to the system
void write2sys(const oc::SimpleSetupPtr problem, const std::vector<Agent*> agents, const std::string& problem_name)
{
    fs::path sol_dir = "solutions/" + GetCurrentTimeForFileName();
    fs::create_directories(sol_dir);
    auto filePath = fs::current_path() / sol_dir / fs::path("problem.yml");
    std::ifstream src("problems/" + problem_name + ".yml", std::ios::binary);
    std::ofstream dst(filePath, std::ios::binary);
    dst << src.rdbuf();
    for (auto agent : agents)
    {
        std::string fileName = agent->getName() + ".txt";
        filePath = fs::current_path() / sol_dir / fs::path(fileName); /// appendTimeToFileName(fileName); // e.g. MyPrettyFile_2018-06-09_01-42-00.txt
        std::ofstream file(filePath);
        const oc::PathControl p = problem->getSolutionPath();
        // p.printAsMatrix(file); // save output as concatenated [state, control, duration] matrix
        p.asGeometric().printAsMatrix(file); // save output as states only at each propogation step size
    }

}
