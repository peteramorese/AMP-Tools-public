#ifndef AMP_EXCLUDE_VIS

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
std::unique_ptr<ampprivate::pybridge::PythonObject> polygonsToPythonObject(const std::vector<Polygon>& polygons) {
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> python_object_ptrs;
    python_object_ptrs.reserve(polygons.size());
    for (const Polygon& polygon : polygons) {
        ampprivate::pybridge::ListOfPairs<double> list_of_doubles;
        const std::vector<Eigen::Vector2d>& vertices = polygon.verticesCCW();
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

std::unique_ptr<ampprivate::pybridge::PythonObject> listOfPointsToPythonObject(const std::vector<Eigen::Vector2d>& list_of_points) {
    ampprivate::pybridge::ListOfPairs<double> list_of_doubles;
    list_of_doubles.list_of_tuples.reserve(list_of_points.size());
    for (const auto& pt : list_of_points) {
        list_of_doubles.list_of_tuples.push_back({{pt[0], pt[1]}});
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

void amp::Visualizer::makeFigure(const Problem2D& prob, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points) {
    newFigure();
    createAxes(prob, path, collision_points);
}

void amp::Visualizer::makeFigure(const std::vector<Polygon>& polygons, bool filled) {
    newFigure();
    createAxes(polygons, filled);
}

void amp::Visualizer::makeFigure(const std::vector<Polygon>& polygons, const std::vector<std::string>& labels, bool filled) {
    newFigure();
    createAxes(polygons, labels, filled);
}

void amp::Visualizer::makeFigure(const std::vector<Polygon>& polygons, const std::vector<double>& heights_3d) {
    newFigure();
    createAxes(polygons, heights_3d);
}

void amp::Visualizer::makeFigure(const LinkManipulator2D& link_manipulator, const ManipulatorState& state) {
    newFigure();
    createAxes(link_manipulator, state);
}

void amp::Visualizer::makeFigure(const Environment2D& env, const LinkManipulator2D& link_manipulator, const ManipulatorState& state) {
    newFigure();
    createAxes(env);
    createAxes(link_manipulator, state);
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorState& state) {
    newFigure();
    createAxes(prob);
    createAxes(link_manipulator, state);
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorTrajectory& trajectory) {
    newFigure();
    createAxes(prob);
    double scale = 0.0;
    for (const ManipulatorState& state : trajectory.waypoints) {
        createAxes(link_manipulator, state, &scale);
        scale += 1.0 / static_cast<double>(trajectory.waypoints.size());
    }
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorTrajectory2Link& trajectory) {
    newFigure();
    createAxes(prob);
    double scale = 0.0;
    for (const ManipulatorState2Link& state : trajectory.waypoints) {
        createAxes(link_manipulator, convert(state), &scale);
        scale += 1.0 / static_cast<double>(trajectory.waypoints.size());
    }
}

void amp::Visualizer::makeFigure(const GridCSpace2D& cspace) {
    newFigure();
    createAxes(cspace);
}

void amp::Visualizer::makeFigure(const PotentialFunction2D& potential_function, double x0_min, double x0_max, double x1_min, double x1_max, std::size_t n_grid, double u_min, double u_max) {
    newFigure();
    createAxes(potential_function, x0_min, x0_max, x1_min, x1_max, n_grid, u_min, u_max);
}

void amp::Visualizer::showFigures() {
    ampprivate::pybridge::ScriptCaller::call("FigureHandler", "show_figure", std::make_tuple());
}

void amp::Visualizer::newFigure() {
    ampprivate::pybridge::ScriptCaller::call("FigureHandler", "new_figure", std::make_tuple());
}

void amp::Visualizer::createAxes(const Environment2D& env) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> bounds_arg = workspaceBoundsToPythonObject(env.x_min, env.x_max, env.y_min, env.y_max);
    std::unique_ptr<ampprivate::pybridge::PythonObject> obstacles_arg = polygonsToPythonObject(env.obstacles);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "visualize_environment", std::make_tuple(bounds_arg->get(), obstacles_arg->get()));
}
void amp::Visualizer::createAxes(const Problem2D& prob) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> bounds_arg = workspaceBoundsToPythonObject(prob.x_min, prob.x_max, prob.y_min, prob.y_max);
    std::unique_ptr<ampprivate::pybridge::PythonObject> obstacles_arg = polygonsToPythonObject(prob.obstacles);
    std::unique_ptr<ampprivate::pybridge::PythonObject> q_init_arg = pointToPythonObject(prob.q_init);
    std::unique_ptr<ampprivate::pybridge::PythonObject> q_goal_arg = pointToPythonObject(prob.q_goal);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "visualize_environment", std::make_tuple(bounds_arg->get(), obstacles_arg->get(), q_init_arg->get(), q_goal_arg->get()));

}

void amp::Visualizer::createAxes(const Problem2D& prob, const Path2D& path) {
    createAxes(prob);
    std::unique_ptr<ampprivate::pybridge::PythonObject> path_arg = listOfPointsToPythonObject(path.waypoints);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "visualize_path", std::make_tuple(path_arg->get()));
}

void amp::Visualizer::createAxes(const Problem2D& prob, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points) {
    createAxes(prob);
    std::unique_ptr<ampprivate::pybridge::PythonObject> path_arg = listOfPointsToPythonObject(path.waypoints);
    std::unique_ptr<ampprivate::pybridge::PythonObject> collison_points_arg = listOfPointsToPythonObject(collision_points);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "visualize_path", std::make_tuple(path_arg->get(), collison_points_arg->get()));
}

void amp::Visualizer::createAxes(const std::vector<Polygon>& polygons, bool filled) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> polygons_arg = polygonsToPythonObject(polygons);
    std::unique_ptr<ampprivate::pybridge::PythonObject> filled_arg = ampprivate::pybridge::makeBool(filled);
    ampprivate::pybridge::ScriptCaller::call("VisualizePolygons", "visualize_polygons", std::make_tuple(polygons_arg->get(), filled_arg->get()));
}

void amp::Visualizer::createAxes(const std::vector<Polygon>& polygons, const std::vector<std::string>& labels, bool filled) {
    ASSERT(polygons.size() == labels.size(), "Number of polygons does not match number of labels");

    // Polygons
    std::unique_ptr<ampprivate::pybridge::PythonObject> polygons_arg = polygonsToPythonObject(std::move(polygons));

    // Labels
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> python_object_ptrs;
    python_object_ptrs.reserve(labels.size());
    for (const auto& label : labels) {
        python_object_ptrs.push_back(ampprivate::pybridge::makeString(label));
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> labels_arg = ampprivate::pybridge::makeList(std::move(python_object_ptrs));

    // Filled
    std::unique_ptr<ampprivate::pybridge::PythonObject> filled_arg = ampprivate::pybridge::makeBool(filled);

    ampprivate::pybridge::ScriptCaller::call("VisualizePolygons", "visualize_polygons", std::make_tuple(polygons_arg->get(), filled_arg->get(), labels_arg->get()));
}

void amp::Visualizer::createAxes(const std::vector<Polygon>& polygons, const std::vector<double>& heights_3d) {
    ASSERT(polygons.size() == heights_3d.size(), "Number of polygons does not match number of heights in heights_3d");
    
    // Polygons
    std::unique_ptr<ampprivate::pybridge::PythonObject> polygons_arg = polygonsToPythonObject(polygons);

    // Heights
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> python_object_ptrs;
    python_object_ptrs.reserve(heights_3d.size());
    for (auto height : heights_3d) {
        python_object_ptrs.push_back(ampprivate::pybridge::makeScalar(height));
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> heights_3d_arg = ampprivate::pybridge::makeList(std::move(python_object_ptrs));

    ampprivate::pybridge::ScriptCaller::call("VisualizePolygons", "visualize_polygons_3d", std::make_tuple(polygons_arg->get(), heights_3d_arg->get()));
}

void amp::Visualizer::createAxes(const LinkManipulator2D& link_manipulator, const ManipulatorState& state, double* cmap_scale) {
    // Get the coordinate for every joint
    std::vector<Eigen::Vector2d> vertices(link_manipulator.nLinks() + 1);
    for (uint32_t joint_index = 0; joint_index < link_manipulator.nLinks() + 1; ++joint_index) {
        vertices[joint_index] = link_manipulator.getJointLocation(state, joint_index);
    }

    std::unique_ptr<ampprivate::pybridge::PythonObject> vertices_arg = listOfPointsToPythonObject(vertices);

    if (cmap_scale) {
        std::unique_ptr<ampprivate::pybridge::PythonObject> alpha_arg = ampprivate::pybridge::makeScalar(*cmap_scale);
        ampprivate::pybridge::ScriptCaller::call("VisualizeLinkManipulator", "visualize_manipulator", std::make_tuple(vertices_arg->get(), alpha_arg->get()));
    } else {
        ampprivate::pybridge::ScriptCaller::call("VisualizeLinkManipulator", "visualize_manipulator", std::make_tuple(vertices_arg->get()));
    }
}

void amp::Visualizer::createAxes(const GridCSpace2D& cspace) {
    auto[x0_cells, x1_cells] = cspace.size();
    std::unique_ptr<ampprivate::pybridge::PythonObject> x0_cells_arg = ampprivate::pybridge::makeLong(x0_cells);
    std::unique_ptr<ampprivate::pybridge::PythonObject> x1_cells_arg = ampprivate::pybridge::makeLong(x1_cells);
    
    // Bounds
    auto[x0_min, x0_max] = cspace.x0Bounds();
    auto[x1_min, x1_max] = cspace.x1Bounds();
    std::unique_ptr<ampprivate::pybridge::PythonObject> bounds_arg = workspaceBoundsToPythonObject(x0_min, x0_max, x1_min, x1_max);

    // Data
    const std::vector<bool>& data = cspace.data();
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> data_python_object_ptrs;
    data_python_object_ptrs.reserve(data.size());
    for (auto bit : data) {
        data_python_object_ptrs.push_back(ampprivate::pybridge::makeBool(bit));
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> data_arg = ampprivate::pybridge::makeList(std::move(data_python_object_ptrs));

    ampprivate::pybridge::ScriptCaller::call("VisualizeCSpace", "visualize_grid_cspace_2d", std::make_tuple(x0_cells_arg->get(), x1_cells_arg->get(), bounds_arg->get(), data_arg->get()));
}

void amp::Visualizer::createAxes(const PotentialFunction2D& potential_function, double x0_min, double x0_max, double x1_min, double x1_max, std::size_t n_grid, double u_min, double u_max) {
    // Bounds
    std::unique_ptr<ampprivate::pybridge::PythonObject> bounds_arg = workspaceBoundsToPythonObject(x0_min, x0_max, x1_min, x1_max); 

    // Grid cells
    std::unique_ptr<ampprivate::pybridge::PythonObject> n_grid_arg = ampprivate::pybridge::makeLong(n_grid);

    // U values
    Eigen::Vector2d lower_left(x0_min, x1_min);
    Eigen::Vector2d upper_right(x0_max, x1_max);

    Eigen::Vector2d disc_diag = 1.0 / static_cast<double>(n_grid) * (upper_right - lower_left);

    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> u_values;
    u_values.reserve(n_grid * n_grid);
    for (std::size_t i = 0; i < n_grid; ++i) {
        for (std::size_t j = 0; j < n_grid; ++j) {
            Eigen::Vector2d coord = lower_left + Eigen::Vector2d(static_cast<double>(i) * disc_diag[0], static_cast<double>(j) * disc_diag[1]);
            double u = potential_function(coord);
            u = std::min(u, u_max); // Clamp below u_max
            u = std::max(u, u_min); // Clamp above u_min
            u_values.push_back(ampprivate::pybridge::makeScalar(u));
        }
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> u_values_arg = ampprivate::pybridge::makeList(std::move(u_values));

    ampprivate::pybridge::ScriptCaller::call("VisualizePotentialFunction", "visualize_potential_function", std::make_tuple(bounds_arg->get(), n_grid_arg->get(), u_values_arg->get()));
}

#endif