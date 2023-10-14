#pragma once

#include "tools/Environment.h"
#include "tools/Obstacle.h"
#include "tools/LinkManipulator.h"
#include "tools/ConfigurationSpace.h"
#include "tools/PotentialFunction.h"

#ifndef AMP_EXCLUDE_VIS

namespace amp {

class Visualizer {
    public:
        /// @brief Visualize an environment. Shows matplotlib figure (blocking until figure is closed)
        /// @param env Environment to display
        static void makeFigure(const Environment2D& env);

        /// @brief Visualize a problem (environment and initial/goal states). Shows matplotlib figure (blocking until figure is closed)
        /// @param prob Problem to display
        static void makeFigure(const Problem2D& prob);

        /// @brief Visualize a problem and a path on the same figure. Shows matplotlib figure (blocking until figure is closed)
        /// @param prob Problem to display
        /// @param path Path to display
        static void makeFigure(const Problem2D& prob, const Path2D& path);

        /// @brief Visualize a problem, path, and any collision points on the same figure. Shows matplotlib figure (blocking until figure is closed)
        /// @param prob Problem to display
        /// @param path Path to display
        /// @param collision_points Collision points to display
        static void makeFigure(const Problem2D& prob, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points);

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

        /// @brief Visualize a grid cspace
        /// @param cspace Dense grid cspace
        static void makeFigure(const GridCSpace2D& cspace);

        /// @brief Visualize a 2D potential function using a 3D height map
        /// @param potential_function Your potential function to visualize
        /// @param x0_min Lower bound for the first dimension
        /// @param x0_max Upper bound for the first dimension
        /// @param x1_min Lower bound for the second dimension
        /// @param x1_max Upper bound for the second dimension
        /// @param n_grid Number of grid cells along each dimension to discretize the height map
        /// @param u_min Lower bound for the height map
        /// @param u_max Upper bound for the height map
        static void makeFigure(const PotentialFunction2D& potential_function, 
                                double x0_min, double x0_max, double x1_min, double x1_max, std::size_t n_grid = 100, double u_min = 0.0, double u_max = 100.0);

        /// @brief Show all figures that were created with `makeFigure()`
        static void showFigures();

    private:
        static void createAxes(const Environment2D& env);
        static void createAxes(const Problem2D& prob);
        static void createAxes(const Problem2D& prob, const Path2D& path);
        static void createAxes(const Problem2D& prob, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points);
        static void createAxes(const std::vector<Polygon>& polygons, bool filled);
        static void createAxes(const std::vector<Polygon>& polygons, const std::vector<std::string>& labels, bool filled);
        static void createAxes(const std::vector<Polygon>& polygons, const std::vector<double>& heights_3d);
        static void createAxes(const LinkManipulator2D& link_manipulator, const ManipulatorState& state, double* cmap_scale = nullptr);
        static void createAxes(const GridCSpace2D& cspace);
        static void createAxes(const PotentialFunction2D& potential_function, double x0_min, double x0_max, double x1_min, double x1_max, std::size_t n_grid, double u_min, double u_max);
        static void newFigure();
};
}

#else

namespace amp {

class Visualizer {
    public:
        static void makeFigure(const Environment2D& env) {}
        static void makeFigure(const Problem2D& prob) {}
        static void makeFigure(const Problem2D& prob, const Path2D& path) {}
        static void makeFigure(const Problem2D& prob, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points) {}
        static void makeFigure(const std::vector<Polygon>& polygons, bool filled = true) {}
        static void makeFigure(const std::vector<Polygon>& polygons, const std::vector<std::string>& labels, bool filled = true) {}
        static void makeFigure(const std::vector<Polygon>& polygons, const std::vector<double>& heights_3d) {}
        static void makeFigure(const LinkManipulator2D& link_manipulator, const ManipulatorState& state) {}
        static void makeFigure(const Environment2D& env, const LinkManipulator2D& link_manipulator, const ManipulatorState& state) {}
        static void makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorState& state) {}
        static void makeFigure(const GridCSpace2D& cspace) {}
        static void makeFigure(const PotentialFunction2D& potential_function, double x0_min, double x0_max, double x1_min, double x1_max, double u_min = 0.0, double u_max = 100.0) {}
        static void showFigures() {}
};
}

#endif
