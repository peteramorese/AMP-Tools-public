#pragma once

#include "tools/Environment.h"
#include "tools/Obstacle.h"
#include "tools/LinkManipulator.h"
#include "tools/ConfigurationSpace.h"
#include "tools/PotentialFunction.h"
#include "tools/Graph.h"

#ifndef AMP_EXCLUDE_VIS

namespace amp {

class Visualizer {
    public:
        /// @brief Visualize an environment. 
        /// @param env Environment to display
        static void makeFigure(const Environment2D& env);

        /// @brief Visualize a problem (environment and initial/goal states). 
        /// @param prob Problem to display
        static void makeFigure(const Problem2D& prob);

        /// @brief Visualize a multi-agent problem (environment and initial/goal states for each agent). 
        /// @param prob Problem to display
        static void makeFigure(const MultiAgentProblem2D& prob);

        /// @brief Visualize a problem and a path on the same figure. 
        /// @param prob Problem to display
        /// @param path Path to display
        static void makeFigure(const Problem2D& prob, const Path2D& path);

        /// @brief Visualize a problem, path, and any collision points on the same figure. 
        /// @param prob Problem to display
        /// @param path Path to display
        /// @param collision_points Collision points to display
        static void makeFigure(const Problem2D& prob, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points);

        /// @brief Visualize a problem and a path on the same figure for a kinodynamic problem w/ rectangular body.
        /// @param prob Problem to display
        /// @param path Path to display
        /// @param agent_dim Dimensions of the rectangular agent
        static void makeFigure(const KinodynamicProblem2D& prob, const amp::KinoPath& path, bool animate = false);

        /// @brief Visualize a problem and a path on the same figure for a circular robot. 
        /// @param env Environment to display
        /// @param path Path to display
        static void makeFigure(const Environment2D& env, const CircularAgentProperties& circular_agent_props, const Path2D& path);

        /// @brief Visualize a problem, path, and any collision states for a circular robot on the same figure. 
        /// @param env Environment to display
        /// @param path Path to display
        /// @param collision_points Collision points to display
        static void makeFigure(const Environment2D& env, const CircularAgentProperties& circular_agent_props, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_states);

        static void makeFigure(const MultiAgentProblem2D& prob, const amp::MultiAgentPath2D& ma_path);
        static void makeFigure(const MultiAgentProblem2D& prob, const amp::MultiAgentPath2D& ma_path, const std::vector<std::vector<Eigen::Vector2d>>& ma_collision_states);

        /// @brief Visualize a set of polygons. Automatically adjusts axis bounds
        /// @param obstacles Obstacles to display, shown in order
        static void makeFigure(const std::vector<Polygon>& polygons, bool filled = true);

        /// @brief Visualize a set of polygons. Automatically adjusts axis bounds
        /// @param obstacles Obstacles to display, shown in order
        /// @param labels Labels corresponding to each obstacle (must be the same size as `obstacles`)
        static void makeFigure(const std::vector<Polygon>& polygons, const std::vector<std::string>& labels, bool filled = true);

        /// @brief Visualize a set of polygons in 3D. Automatically adjusts axis bounds
        /// @param obstacles Obstacles to display, shown in order
        /// @param heights_3d Z-values (heights) corresponding to each obstacle (must be the same size as `obstacles`)
        static void makeFigure(const std::vector<Polygon>& polygons, const std::vector<double>& heights_3d);

        /// @brief Visualize a link manipulator.
        /// NOTE: The manipulator is visualized with a non-zero thickness even though the links have zero thickness (lines)
        /// @param link_manipulator Manipulator
        /// @param state State of the manipulator to display
        static void makeFigure(const LinkManipulator2D& link_manipulator, const ManipulatorState& state);

        /// @brief Visualize a link manipulator.
        /// NOTE: The manipulator is visualized with a non-zero thickness even though the links have zero thickness (lines)
        /// @param env Environment to display
        /// @param link_manipulator Manipulator
        /// @param state State of the manipulator to display
        static void makeFigure(const Environment2D& env, const LinkManipulator2D& link_manipulator, const ManipulatorState& state);

        /// @brief Visualize a link manipulator.
        /// NOTE: The manipulator is visualized with a non-zero thickness even though the links have zero thickness (lines)
        /// @param prob Problem to display
        /// @param link_manipulator Manipulator
        /// @param state State of the manipulator to display
        static void makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorState& state);

        /// @brief Visualize a link manipulator.
        /// NOTE: The manipulator is visualized with a non-zero thickness even though the links have zero thickness (lines)
        /// @param prob Problem to display
        /// @param link_manipulator Manipulator
        /// @param trajectory Chronological trajectory of the manipulator (overloads provided for 2-link and N-link)
        static void makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorTrajectory& trajectory);
        static void makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorTrajectory2Link& trajectory);

        /// @brief Visualize a link manipulator and show colliding states.
        /// NOTE: The manipulator is visualized with a non-zero thickness even though the links have zero thickness (lines)
        /// @param prob Problem to display
        /// @param link_manipulator Manipulator
        /// @param trajectory Chronological trajectory of the manipulator (overloads provided for 2-link and N-link)
        /// @param collision_states Collision states to display (overloads provided for 2-link and N-link)
        static void makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorTrajectory& trajectory, const std::vector<ManipulatorState>& collision_states);
        static void makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorTrajectory2Link& trajectory, const std::vector<ManipulatorState2Link>& collision_states);

        /// @brief Visualize a grid cspace
        /// @param cspace Dense grid cspace
        static void makeFigure(const GridCSpace2D& cspace);

        /// @brief Visualize a grid cspace
        /// @param cspace Dense grid cspace
        /// @param path Path inside cspace (ManipulatorTrajectory2Link is also accepted alias of Path2D)
        static void makeFigure(const GridCSpace2D& cspace, const Path2D& path);

        /// @brief Visualize a 2D potential function using a 3D height map
        /// @param potential_function Your potential function to visualize
        /// @param x0_min Lower bound for the first dimension
        /// @param x0_max Upper bound for the first dimension
        /// @param x1_min Lower bound for the second dimension
        /// @param x1_max Upper bound for the second dimension
        /// @param n_grid Number of grid cells along each dimension to discretize the height map
        /// @param u_min Lower bound for the height map
        /// @param u_max Upper bound for the height map
        static void makeFigure(const PotentialFunction2D& potential_function,const Problem2D& prob, std::size_t n_grid = 100, bool vector = true, double u_min = 0.0, double u_max = 100.0);

        /// @brief Visualize a 2D problem with a map of coordinates
        /// @tparam FXN Callable type [automatically deduced]
        /// @param prob Problem to display
        /// @param map Coordinate map where nodes represent points on the map with edges connecting them
        /// @param getCoordinateFromNode Callable (lambda or object that has operator()) that returns the coordinate given a node. Signature:
        /// `Eigen::Vector2d getCoordinateFromNode(amp::Node node)`
        template <typename FXN>
        static void makeFigure(const Problem2D& prob, const Graph<double>& coordimate_map, const FXN& getCoordinateFromNode);

        /// @brief Visualize a 2D problem with a map of coordinates
        /// @tparam FXN Callable type [automatically deduced]
        /// @param prob Problem to display
        /// @param path Path to display
        /// @param map Coordinate map where nodes represent points on the map with edges connecting them
        /// @param getCoordinateFromNode Callable (lambda or object that has operator()) that returns the coordinate given a node. Signature:
        /// `Eigen::Vector2d getCoordinateFromNode(amp::Node node)`
        template <typename FXN>
        static void makeFigure(const Problem2D& prob, const Path2D& path, const Graph<double>& coordinate_map, const FXN& getCoordinateFromNode);

        /// @brief Visualize a 2D problem with a map of coordinates
        /// @param prob Problem to display
        /// @param map Coordinate map where nodes represent points on the map with edges connecting them
        /// @param node_to_coordinate Container that maps each node in the graph to its corresponding coordinate
        /// `Eigen::Vector2d getCoordinateFromNode(amp::Node node)`
        static void makeFigure(const Problem2D& prob, const Graph<double>& coordinate_map, const std::map<amp::Node, Eigen::Vector2d>& node_to_coordinate);

        /// @brief Visualize a 2D problem with a map of coordinates
        /// @param prob Problem to display
        /// @param path Path to display
        /// @param map Coordinate map where nodes represent points on the map with edges connecting them
        /// @param node_to_coordinate Container that maps each node in the graph to its corresponding coordinate
        /// `Eigen::Vector2d getCoordinateFromNode(amp::Node node)`
        static void makeFigure(const Problem2D& prob, const Path2D& path, const Graph<double>& coordinate_map, const std::map<amp::Node, Eigen::Vector2d>& node_to_coordinate);

        /// @brief Make a box plot figure to display benchmark results.
        /// @param data_sets List of data sets that each contain all the data for a single category (i.e. computation time)
        /// @param labels Labels for each data set (must be same size as data_sets)
        /// @param title Title of the plot
        /// @param xlabel X-axis label
        /// @param ylabel Y-axis label
        static void makeBoxPlot(const std::list<std::vector<double>>& data_sets, const std::vector<std::string>& labels, 
                                const std::string& title = std::string(), const std::string& xlabel = std::string(), const std::string& ylabel = std::string());

        /// @brief Make a bar graph figure to display benchmark results.
        /// @param values Values corresponding to each label (must be same size as labels)
        /// @param labels Labels for each value
        /// @param title Title of the plot
        /// @param xlabel X-axis label
        /// @param ylabel Y-axis label
        static void makeBarGraph(const std::vector<double>& values, const std::vector<std::string>& labels, 
                                const std::string& title = std::string(), const std::string& xlabel = std::string(), const std::string& ylabel = std::string());

        /// @brief Save and show all figures that were created with `makeFigure()`
        /// @param show Make the figures pop up in GUI
        /// @param directory Directory relative to the root of the project to save the figures in
        /// @param format File extension format to save the figure in, e.g. png, pdf
        static void saveFigures(bool show = true, const std::string& directory = "all", const std::string& format = "png");

    private:
        static void saveFigures(const std::string& format, const std::string& directory);
        static void createAxes(const Environment2D& env);
        static void createAxes(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal);
        static void createAxes(const Problem2D& prob);
        static void createAxes(const MultiAgentProblem2D& prob);
        static void createAxes(const Path2D& path);
        static void createAxes(const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points);
        static void createAxes(const KinoPath& path, const AgentDimensions& agent_dim, bool isCar, bool animate);
        static void createAxes(const Eigen::VectorXd& q_init, const std::vector<std::pair<double, double>>& q_goal);
        static void createAxes(double circular_agent_radius, const Eigen::Vector2d& state, double* cmap_scale = nullptr, bool colliding = false);
        static void createAxes(double circular_agent_radius, const Path2D& path, bool random_color = false, const std::vector<Eigen::Vector2d>* collision_states = nullptr);
        static void createAxes(const std::vector<Polygon>& polygons, bool filled);
        static void createAxes(const std::vector<Polygon>& polygons, const std::vector<std::string>& labels, bool filled);
        static void createAxes(const std::vector<Polygon>& polygons, const std::vector<double>& heights_3d);
        static void createAxes(const LinkManipulator2D& link_manipulator, const ManipulatorState& state, double* cmap_scale = nullptr, bool colliding = false);
        static void createAxes(const GridCSpace2D& cspace);
        static void createAxes(const PotentialFunction2D& potential_function, const Problem2D& prob, std::size_t n_grid, bool vector, double u_min, double u_max);
        template <typename FXN>
        static void createAxes(const Graph<double>& map, const FXN& getCoordinateFromNode);
        static void newFigure();
};
}

#include "public/Visualizer_impl.h"

#else

namespace amp {

class Visualizer {
    public:
        static void makeFigure(const Environment2D& env) {}
        static void makeFigure(const Problem2D& prob) {}
        static void makeFigure(const MultiAgentProblem2D& prob) {}
        static void makeFigure(const Problem2D& prob, const Path2D& path) {}
        static void makeFigure(const Problem2D& prob, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points) {}
        static void makeFigure(const Problem2D& prob, double circular_agent_radius, const Path2D& path) {}
        static void makeFigure(const Problem2D& prob, double circular_agent_radius, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_states) {}
        static void makeFigure(const std::vector<Polygon>& polygons, bool filled = true) {}
        static void makeFigure(const std::vector<Polygon>& polygons, const std::vector<std::string>& labels, bool filled = true) {}
        static void makeFigure(const std::vector<Polygon>& polygons, const std::vector<double>& heights_3d) {}
        static void makeFigure(const LinkManipulator2D& link_manipulator, const ManipulatorState& state) {}
        static void makeFigure(const Environment2D& env, const LinkManipulator2D& link_manipulator, const ManipulatorState& state) {}
        static void makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorState& state) {}
        static void makeFigure(const GridCSpace2D& cspace) {}
        static void makeFigure(const GridCSpace2D& cspace, const Path2D& path) {}
        static void makeFigure(const PotentialFunction2D& potential_function, double x0_min, double x0_max, double x1_min, double x1_max, double u_min = 0.0, double u_max = 100.0) {}
        template <typename FXN>
        static void makeFigure(const Problem2D& prob, const Graph<double>& map, const FXN& getCoordinateFromNode) {}
        static void makeBoxPlot(const std::list<std::vector<double>>& data_sets, const std::vector<std::string>& labels, 
                                const std::string& title = std::string(), const std::string& xlabel = std::string(), const std::string& ylabel = std::string()) {}
        static void makeBarGraph(const std::vector<double>& values, const std::vector<std::string>& labels, 
                                const std::string& title = std::string(), const std::string& xlabel = std::string(), const std::string& ylabel = std::string()) {}
        static void showFigures() {}
};
}

#endif
