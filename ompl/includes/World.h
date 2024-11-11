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

#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <ompl/tools/config/SelfConfig.h>
#include <yaml-cpp/yaml.h>


typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

// rectangular Obstacle defined by two points (xMin, yMin) and (xMax, yMax)
struct Obstacle {
    const double xMin_;
    const double yMin_;
    const double xMax_;
    const double yMax_;
    const polygon poly_;

    void printPoints() const
    {
        auto exterior_points = boost::geometry::exterior_ring(poly_);
        for (int i=0; i<exterior_points.size(); i++)
        {
            std::cout << boost::geometry::get<0>(exterior_points[i]) << " " << boost::geometry::get<1>(exterior_points[i]) <<std::endl;
        }
    }
    polygon::ring_type getPoints() const
    {
        return boost::geometry::exterior_ring(poly_);
    }
};


// An agent has a name, shape, dynamics, start and goal regions
// Created as class to keep important variables safe
class Agent
{
public:
    Agent(std::string name, std::string dyn, std::vector<double> shape, std::vector<double> s, std::vector<double> g) {
        name_ = name;
        dynamics_ = dyn;
        shape_ = shape;
        start_ = s;
        goal_ = g;
    }
    std::string getName() const {return name_;};
    std::string getDynamics() const {return dynamics_;};
    std::vector<double> getShape() const {return shape_;};
    std::vector<double> getStartLocation() const {return start_;};
    std::vector<double> getGoalLocation() const {return goal_;};
private:
    std::string name_;
    std::string dynamics_;
    std::vector<double> shape_;
    std::vector<double> start_;
    std::vector<double> goal_;
};



// world class holds all relevent data in the world that is used by OMPL
class World
{
public:
    World(){}
    // methods for dimensions
    void setWorldDimensions(std::pair<double, double> x, std::pair<double, double> y){xBounds_ = x; yBounds_ = y;};
    std::vector<std::pair<double, double>> getWorldDimensions() const {return {xBounds_, yBounds_};};
    void printWorldDimensions(){OMPL_INFORM("Space Dimensions: [%0.2f, %0.2f, %0.2f, %0.2f]", xBounds_.first, yBounds_.first, xBounds_.second, yBounds_.second);}
    // methods for obstacles
    void addObstacle(Obstacle obs){Obstacles_.push_back(obs);};
    std::vector<Obstacle> getObstacles() const {return Obstacles_;};
    // methods for agents
    void addAgent(Agent *a){Agents_.push_back(a);};
    std::vector<Agent*> getAgents() const {return Agents_;};
    // printing methods for usability
    void printObstacles()
    {
        OMPL_INFORM("%d Obstacle(s) (xMin, yMin, xMax, yMax): ", Obstacles_.size());
        for (Obstacle o: Obstacles_)
        {
            OMPL_INFORM("   - Obstacle: [%0.2f, %0.2f, %0.2f, %0.2f]", o.xMin_, o.yMin_, o.xMax_, o.yMax_);
        }
    }
    void printAgents()
    {
        OMPL_INFORM("%d Agents: ", Agents_.size());
        for (Agent *a: Agents_)
        {
            OMPL_INFORM("   - Name: %s", a->getName().c_str());
            OMPL_INFORM("     Dynamics: %s", a->getDynamics().c_str());
            OMPL_INFORM("     Start: [%0.2f, %0.2f]", a->getStartLocation()[0], a->getStartLocation()[1]);
            OMPL_INFORM("     Goal: [%0.2f, %0.2f]", a->getGoalLocation()[0], a->getGoalLocation()[1]);
        }
    }
    void printWorld()
    {
        printWorldDimensions();
        printObstacles();
        printAgents();
    }
private:
    std::pair<double, double> xBounds_;
    std::pair<double, double> yBounds_;
    std::vector<Obstacle> Obstacles_;
    std::vector<Agent*> Agents_;
};

// function that parses YAML file to world object
World* yaml2world(std::string file) {
    YAML::Node config;
    World *w = new World();
    try {
        OMPL_INFORM("Path to Problem File: %s", file.c_str());
        config = YAML::LoadFile(file);
        std::cout << "" << std::endl;
        OMPL_INFORM("File loaded successfully. Parsing...");
    } catch (const std::exception& e) {
        OMPL_ERROR("Invalid file path. Aborting prematurely to avoid critical error.");
        exit(1);
    } try {        
        // grab dimensions from problem definition
        const auto& dims = config["Map"]["Dimensions"];
        const double x_min = dims[0].as<double>();
        const double y_min = dims[1].as<double>();
        const double x_max = dims[2].as<double>();
        const double y_max = dims[3].as<double>();
        w->setWorldDimensions({x_min, x_max}, {y_min, y_max});
   
        // set Obstacles
        const auto& obs = config["Map"]["Obstacles"];
        for (int i=0; i < obs.size(); i++) {
            std::string name = "obstacle" + std::to_string(i);
            const double minX = obs[name][0].as<double>();
            const double minY = obs[name][1].as<double>();
            const double maxX = obs[name][2].as<double>();
            const double maxY = obs[name][3].as<double>();            
            // TOP RIGHT VERTEX:
            std::string top_right = std::to_string(maxX) + " " + std::to_string(maxY);
            // TOP LEFT VERTEX:
            std::string top_left = std::to_string(minX) + " " + std::to_string(maxY);
            // BOTTOM LEFT VERTEX:
            std::string bottom_left = std::to_string(minX) + " " + std::to_string(minY);
            // BOTTOM RIGHT VERTEX:
            std::string bottom_right = std::to_string(maxX) + " " + std::to_string(minY);

            // convert to string for easy initializataion
            std::string points = "POLYGON((" + bottom_left + "," + bottom_right + "," + top_right + "," + top_left + "," + bottom_left + "))";
            polygon poly;
            boost::geometry::read_wkt(points,poly);
            Obstacle o = {minX, minY, maxX, maxY, poly};
            w->addObstacle(o);
        }

        // Define the agents
        const auto& agents = config["Agents"];
        for (int i=0; i < agents.size(); i++) {
            std::string name = "agent" + std::to_string(i);
            const std::vector<double> shape{agents[name]["Shape"][0].as<double>(), agents[name]["Shape"][1].as<double>()};
            const std::vector<double> start{agents[name]["Start"][0].as<double>(), agents[name]["Start"][1].as<double>()};
            const std::vector<double> goal{agents[name]["Goal"][0].as<double>(), agents[name]["Goal"][1].as<double>()};
            Agent *a = new Agent(name, agents[name]["Model"].as<std::string>(), shape, start, goal);
            w->addAgent(a);
        }
        OMPL_INFORM("Parsing Complete.");
        std::cout << "" << std::endl;
        w->printWorld();
    } catch (const std::exception& e) {
        OMPL_ERROR("Error During Parsing. Aborting prematurely to avoid critical error.");
        exit(1);
    }
    return w;
}
