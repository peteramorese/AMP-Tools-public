#include "tools/Visualizer.h"

#include "tools/Logging.h"

#include "public/PythonObjects.h"
#include "public/ScriptCaller.h"

// Local methods for converting things to python object (not in header)
namespace amp {

std::unique_ptr<ampprivate::pybridge::PythonObject> pointToPythonObject(const Eigen::Vector2d& point) {
    std::pair<std::unique_ptr<ampprivate::pybridge::PythonObject>, std::unique_ptr<ampprivate::pybridge::PythonObject>> python_object_pair
        = std::make_pair(ampprivate::pybridge::makeScalar(point[0]), ampprivate::pybridge::makeScalar(point[1]));
    return ampprivate::pybridge::makePair(std::move(python_object_pair));
}
std::unique_ptr<ampprivate::pybridge::PythonObject> obstaclesToPythonObject(const std::vector<Obstacle2D>& obstacles) {
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> python_object_ptrs;
    python_object_ptrs.reserve(obstacles.size());
    for (const Obstacle2D& obstacle : obstacles) {
        ampprivate::pybridge::ListOfPairs<double> list_of_doubles;
        const std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
        for (const Eigen::Vector2d& vertex : vertices) {
            std::array<double, 2> vertex_array;
            vertex_array[0] = vertex[0];
            vertex_array[1] = vertex[1];
            list_of_doubles.list_of_tuples.push_back(vertex_array);
        }
        python_object_ptrs.emplace_back(list_of_doubles.toPyList());
    }

    return ampprivate::pybridge::makeList(std::move(python_object_ptrs));
}

std::unique_ptr<ampprivate::pybridge::PythonObject> workspaceBoundsToPythonObject(double x_min, double x_max, double y_min, double y_max) {
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> python_object_ptrs;
    python_object_ptrs.reserve(4);
    python_object_ptrs.push_back(ampprivate::pybridge::makeScalar(x_min));
    python_object_ptrs.push_back(ampprivate::pybridge::makeScalar(x_max));
    python_object_ptrs.push_back(ampprivate::pybridge::makeScalar(y_min));
    python_object_ptrs.push_back(ampprivate::pybridge::makeScalar(y_max));
    return ampprivate::pybridge::makeList(std::move(python_object_ptrs));
}

std::unique_ptr<ampprivate::pybridge::PythonObject> pathToPythonObject(const Path2D& path) {
    ampprivate::pybridge::ListOfPairs<double> list_of_doubles;
    list_of_doubles.list_of_tuples.reserve(path.waypoints.size());
    for (const auto& waypt : path.waypoints) {
        list_of_doubles.list_of_tuples.push_back({{waypt[0], waypt[1]}});
    }
    return list_of_doubles.toPyList();
}

} // namespace amp

void amp::Visualizer::makeFigure(const Environment2D& env) {
    newFigure();
    createAxes(env);
}

void amp::Visualizer::makeFigure(const Problem2D& prob) {
    newFigure();
    createAxes(prob);
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const Path2D& path) {
    newFigure();
    createAxes(prob, path);
}

void amp::Visualizer::showFigures() {
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "show_figure", std::make_tuple());
}

void amp::Visualizer::newFigure() {
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "new_figure", std::make_tuple());
}

void amp::Visualizer::createAxes(const Environment2D& env) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> bounds_arg = workspaceBoundsToPythonObject(env.x_min, env.x_max, env.y_min, env.y_max);
    std::unique_ptr<ampprivate::pybridge::PythonObject> obstacles_arg = obstaclesToPythonObject(env.obstacles);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "visualize_environment", std::make_tuple(bounds_arg->get(), obstacles_arg->get()));
}
void amp::Visualizer::createAxes(const Problem2D& prob) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> bounds_arg = workspaceBoundsToPythonObject(prob.x_min, prob.x_max, prob.y_min, prob.y_max);
    std::unique_ptr<ampprivate::pybridge::PythonObject> obstacles_arg = obstaclesToPythonObject(prob.obstacles);
    std::unique_ptr<ampprivate::pybridge::PythonObject> q_init_arg = pointToPythonObject(prob.q_init);
    std::unique_ptr<ampprivate::pybridge::PythonObject> q_goal_arg = pointToPythonObject(prob.q_goal);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "visualize_environment", std::make_tuple(bounds_arg->get(), obstacles_arg->get(), q_init_arg->get(), q_goal_arg->get()));

}

void amp::Visualizer::createAxes(const Problem2D& prob, const Path2D& path) {
    createAxes(prob);
    std::unique_ptr<ampprivate::pybridge::PythonObject> path_arg = pathToPythonObject(path);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "visualize_path", std::make_tuple(path_arg->get()));
}