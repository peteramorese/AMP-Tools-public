#pragma once

#include "Environment.h"

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

        /// @brief Show all figures that were created with `makeFigure()`
        static void showFigures();

    private:
        static void createAxes(const Environment2D& env);
        static void createAxes(const Problem2D& prob);
        static void createAxes(const Problem2D& prob, const Path2D& path);
        static void createAxes(const Problem2D& prob, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points);
        static void newFigure();
};
}