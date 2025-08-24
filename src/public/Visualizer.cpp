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

std::unique_ptr<ampprivate::pybridge::PythonObject> listOfPointsToPythonObject(const std::vector<Eigen::VectorXd>& list_of_points) {
    ampprivate::pybridge::ListOfTriples<double> list_of_doubles;
    list_of_doubles.list_of_tuples.reserve(list_of_points.size());
    for (const auto& pt : list_of_points) {
        std::array<double, 3> point = {pt[0], pt[1], pt[2]};
        list_of_doubles.list_of_tuples.push_back({point});
    }
    return list_of_doubles.toPyList();
}

std::unique_ptr<ampprivate::pybridge::PythonObject> listOfPairsToPythonObject(const std::vector<std::pair<double, double>>& list_of_pairs) {
    ampprivate::pybridge::ListOfPairs<double> list_of_doubles;
    list_of_doubles.list_of_tuples.reserve(list_of_pairs.size());
    for (const auto& pt : list_of_pairs) {
        list_of_doubles.list_of_tuples.push_back({{pt.first, pt.second}});
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

void amp::Visualizer::makeFigure(const MultiAgentProblem2D& prob) {
    newFigure();
    createAxes(prob);
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const Path2D& path) {
    newFigure();
    createAxes(prob);
    createAxes(path);
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points) {
    newFigure();
    createAxes(prob);
    createAxes(path, collision_points);
}

void amp::Visualizer::makeFigure(const Environment2D& env, const CircularAgentProperties& circular_agent_props, const Path2D& path) {
    newFigure();
    createAxes(circular_agent_props.radius, path, true);
    createAxes(env);
    createAxes(circular_agent_props.q_init, circular_agent_props.q_goal);
}

void amp::Visualizer::makeFigure(const Environment2D& env, const CircularAgentProperties& circular_agent_props, const Path2D& path, const std::vector<Eigen::Vector2d>& collision_states) {
    newFigure();
    createAxes(circular_agent_props.radius, path, true);
    createAxes(env);
    createAxes(circular_agent_props.q_init, circular_agent_props.q_goal);
}

void amp::Visualizer::makeFigure(const MultiAgentProblem2D& prob, const amp::MultiAgentPath2D& ma_path) {
    newFigure();
    ASSERT(prob.numAgents() == ma_path.numAgents(), "Number of paths does not match number of agents");
    for (uint32_t i = 0; i < prob.numAgents(); ++i) {
        const CircularAgentProperties& props = prob.agent_properties[i];
        createAxes(props.radius, ma_path.agent_paths[i], true);
        createAxes(props.q_init, props.q_goal);
    }
    createAxes(static_cast<const Environment2D&>(prob));
}

void amp::Visualizer::makeFigure(const KinodynamicProblem2D& prob, const amp::KinoPath& path, bool animate) {
    newFigure();
    createAxes(static_cast<const Environment2D&>(prob));
    createAxes(prob.q_init, prob.q_goal);
    createAxes(path, prob.agent_dim, prob.agent_type == amp::AgentType::SimpleCar, animate);
}

void amp::Visualizer::makeFigure(const MultiAgentProblem2D& prob, const amp::MultiAgentPath2D& ma_path, const std::vector<std::vector<Eigen::Vector2d>>& ma_collision_states) {
    newFigure();
    ASSERT(prob.numAgents() == ma_path.numAgents(), "Number of paths does not match number of agents");
    ASSERT(prob.numAgents() == ma_collision_states.size(), "Number of collision state sets does not match number of agents");
    for (uint32_t i = 0; i < prob.numAgents(); ++i) {
        const CircularAgentProperties& props = prob.agent_properties[i];
        createAxes(props.radius, ma_path.agent_paths[i], true, &ma_collision_states[i]);
        createAxes(props.q_init, props.q_goal);
    }
    createAxes(static_cast<const Environment2D&>(prob));
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
    double scale = 0.0;
    for (const ManipulatorState& state : trajectory.waypoints) {
        createAxes(link_manipulator, state, &scale);
        scale += 1.0 / static_cast<double>(trajectory.waypoints.size());
    }
    createAxes(prob);
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorTrajectory2Link& trajectory) {
    newFigure();
    double scale = 0.0;
    for (const ManipulatorState2Link& state : trajectory.waypoints) {
        createAxes(link_manipulator, convert(state), &scale);
        scale += 1.0 / static_cast<double>(trajectory.waypoints.size());
    }
    createAxes(prob);
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorTrajectory& trajectory, const std::vector<ManipulatorState>& collision_states) {
    newFigure();
    for (const ManipulatorState& state : trajectory.waypoints) {
        createAxes(link_manipulator, state);
    }
    createAxes(prob);
    for (const ManipulatorState& colliding_state : collision_states) {
        createAxes(link_manipulator, colliding_state, nullptr, true);
    }
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const LinkManipulator2D& link_manipulator, const ManipulatorTrajectory2Link& trajectory, const std::vector<ManipulatorState2Link>& collision_states) {
    newFigure();
    for (const ManipulatorState2Link& state : trajectory.waypoints) {
        createAxes(link_manipulator, convert(state));
    }
    createAxes(prob);
    for (const ManipulatorState2Link& colliding_state : collision_states) {
        createAxes(link_manipulator, convert(colliding_state), nullptr, true);
    }
}

void amp::Visualizer::makeFigure(const GridCSpace2D& cspace) {
    newFigure();
    createAxes(cspace);
}

void amp::Visualizer::makeFigure(const GridCSpace2D& cspace, const Path2D& path) {
    newFigure();
    createAxes(cspace);
    createAxes(path);
}

void amp::Visualizer::makeFigure(const PotentialFunction2D& potential_function, const Problem2D& prob, std::size_t n_grid, bool vector, double u_min, double u_max) {
    newFigure();
    createAxes(prob);
    createAxes(potential_function, prob, n_grid, vector, u_min, u_max);
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const Graph<double>& coordinate_map, const std::map<amp::Node, Eigen::Vector2d>& node_to_coordinate) {
    createAxes(prob);
    createAxes(coordinate_map, [&](amp::Node node) -> Eigen::Vector2d {return node_to_coordinate.at(node);});
}

void amp::Visualizer::makeFigure(const Problem2D& prob, const Path2D& path, const Graph<double>& coordinate_map, const std::map<amp::Node, Eigen::Vector2d>& node_to_coordinate) {
    newFigure();
    createAxes(prob);
    createAxes(coordinate_map, [&](amp::Node node) -> Eigen::Vector2d {return node_to_coordinate.at(node);});
    createAxes(path);
}

void amp::Visualizer::saveFigures(bool show, const std::string& directory, const std::string& format) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> directory_arg = ampprivate::pybridge::makeString(directory);
    std::unique_ptr<ampprivate::pybridge::PythonObject> format_arg = ampprivate::pybridge::makeString(format);
    ampprivate::pybridge::ScriptCaller::call("FigureHandler", "save_figures", std::make_tuple(directory_arg->get(), format_arg->get()));
    if (show) {
        ampprivate::pybridge::ScriptCaller::call("FigureHandler", "show_figure", std::make_tuple());
    }
}

void amp::Visualizer::newFigure() {
    ampprivate::pybridge::ScriptCaller::call("FigureHandler", "new_figure", std::make_tuple());
}

void amp::Visualizer::createAxes(const Environment2D& env) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> bounds_arg = workspaceBoundsToPythonObject(env.x_min, env.x_max, env.y_min, env.y_max);
    std::unique_ptr<ampprivate::pybridge::PythonObject> obstacles_arg = polygonsToPythonObject(env.obstacles);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "visualize_environment", std::make_tuple(bounds_arg->get(), obstacles_arg->get()));
}

void amp::Visualizer::createAxes(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> q_init_arg = pointToPythonObject(q_init);
    std::unique_ptr<ampprivate::pybridge::PythonObject> q_goal_arg = pointToPythonObject(q_goal);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "show_endpoints", std::make_tuple(q_init_arg->get(), q_goal_arg->get()));
}

void amp::Visualizer::createAxes(const Problem2D& prob) {
    createAxes(static_cast<const Environment2D&>(prob));
    createAxes(prob.q_init, prob.q_goal);
}

void amp::Visualizer::createAxes(const MultiAgentProblem2D& prob) {
    createAxes(static_cast<const Environment2D&>(prob));

    for (const amp::CircularAgentProperties& props : prob.agent_properties) {
        std::unique_ptr<ampprivate::pybridge::PythonObject> q_init_arg = pointToPythonObject(props.q_init);
        std::unique_ptr<ampprivate::pybridge::PythonObject> q_goal_arg = pointToPythonObject(props.q_goal);
        std::unique_ptr<ampprivate::pybridge::PythonObject> random_color_arg = ampprivate::pybridge::makeBool(true);
        ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "show_endpoints", std::make_tuple(q_init_arg->get(), q_goal_arg->get(), random_color_arg->get()));
    }
}

void amp::Visualizer::createAxes(const Path2D& path) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> path_arg = listOfPointsToPythonObject(path.waypoints);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "visualize_path", std::make_tuple(path_arg->get()));
}

void amp::Visualizer::createAxes(const Path2D& path, const std::vector<Eigen::Vector2d>& collision_points) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> path_arg = listOfPointsToPythonObject(path.waypoints);
    std::unique_ptr<ampprivate::pybridge::PythonObject> collison_points_arg = listOfPointsToPythonObject(collision_points);
    ampprivate::pybridge::ScriptCaller::call("VisualizeEnvironment", "visualize_path", std::make_tuple(path_arg->get(), collison_points_arg->get()));
}

void amp::Visualizer::createAxes(const KinoPath& path, const AgentDimensions& agent_dim, bool isCar, bool animate) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> path_arg = listOfPointsToPythonObject(path.waypoints);
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> python_object_ptrs;
    python_object_ptrs.reserve(path.durations.size());
    for (const auto& duration : path.durations)
        python_object_ptrs.push_back(ampprivate::pybridge::makeScalar(duration));
    std::unique_ptr<ampprivate::pybridge::PythonObject> duration_arg = ampprivate::pybridge::makeList(std::move(python_object_ptrs));
    ampprivate::pybridge::ScriptCaller::call("VisualizeDynamicAgent", "visualize_agent", std::make_tuple(path_arg->get(), duration_arg->get(), ampprivate::pybridge::makeScalar(agent_dim.length)->get(), ampprivate::pybridge::makeScalar(agent_dim.width)->get(), ampprivate::pybridge::makeBool(animate)->get(), ampprivate::pybridge::makeBool(isCar)->get()));
}

void amp::Visualizer::createAxes(const Eigen::VectorXd& q_init, const std::vector<std::pair<double, double>>& q_goal) {
    Eigen::Vector2d q_init_2d = q_init.head<2>();
    std::unique_ptr<ampprivate::pybridge::PythonObject> q_init_obj = pointToPythonObject(q_init_2d);
    std::unique_ptr<ampprivate::pybridge::PythonObject> q_goal_obj = listOfPairsToPythonObject(q_goal);
    ampprivate::pybridge::ScriptCaller::call("VisualizeDynamicAgent", "visualize_goal", std::make_tuple(q_init_obj->get(), q_goal_obj->get()));

}

void amp::Visualizer::createAxes(double circular_agent_radius, const Eigen::Vector2d& state, double* cmap_scale, bool colliding) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> radius_arg = ampprivate::pybridge::makeScalar(circular_agent_radius);
    std::unique_ptr<ampprivate::pybridge::PythonObject> centerpoint_arg = pointToPythonObject(state);
    std::unique_ptr<ampprivate::pybridge::PythonObject> colliding_arg = ampprivate::pybridge::makeBool(colliding);

    if (cmap_scale) {
        std::unique_ptr<ampprivate::pybridge::PythonObject> cmap_scale_arg = ampprivate::pybridge::makeScalar(*cmap_scale);
        ampprivate::pybridge::ScriptCaller::call("VisualizeCircleAgent", "visualize_circle_agent", std::make_tuple(radius_arg->get(), centerpoint_arg->get(), colliding_arg->get(), cmap_scale_arg->get()));
    } else {
        ampprivate::pybridge::ScriptCaller::call("VisualizeCircleAgent", "visualize_circle_agent", std::make_tuple(radius_arg->get(), centerpoint_arg->get(), colliding_arg->get()));
    }
}

void amp::Visualizer::createAxes(double circular_agent_radius, const Path2D& path, bool random_color, const std::vector<Eigen::Vector2d>* collision_states) {
    std::unique_ptr<ampprivate::pybridge::PythonObject> radius_arg = ampprivate::pybridge::makeScalar(circular_agent_radius);
    std::unique_ptr<ampprivate::pybridge::PythonObject> path_arg = listOfPointsToPythonObject(path.waypoints);
    std::unique_ptr<ampprivate::pybridge::PythonObject> random_color_arg = ampprivate::pybridge::makeBool(random_color);

    if (collision_states) {
        std::unique_ptr<ampprivate::pybridge::PythonObject> collision_states_arg = listOfPointsToPythonObject(*collision_states);
        ampprivate::pybridge::ScriptCaller::call("VisualizeCircleAgent", "visualize_path", std::make_tuple(radius_arg->get(), path_arg->get(), random_color_arg->get(), collision_states_arg->get()));
    } else {
        ampprivate::pybridge::ScriptCaller::call("VisualizeCircleAgent", "visualize_path", std::make_tuple(radius_arg->get(), path_arg->get(), random_color_arg->get()));
    }
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

void amp::Visualizer::createAxes(const LinkManipulator2D& link_manipulator, const ManipulatorState& state, double* cmap_scale, bool colliding) {
    // Get the coordinate for every joint
    std::vector<Eigen::Vector2d> vertices(link_manipulator.nLinks() + 1);
    for (uint32_t joint_index = 0; joint_index < link_manipulator.nLinks() + 1; ++joint_index) {
        vertices[joint_index] = link_manipulator.getJointLocation(state, joint_index);
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> vertices_arg = listOfPointsToPythonObject(vertices);

    // Colliding arg
    std::unique_ptr<ampprivate::pybridge::PythonObject> colliding_arg = ampprivate::pybridge::makeBool(colliding);

    if (cmap_scale) {
        std::unique_ptr<ampprivate::pybridge::PythonObject> alpha_arg = ampprivate::pybridge::makeScalar(*cmap_scale);
        ampprivate::pybridge::ScriptCaller::call("VisualizeLinkManipulator", "visualize_manipulator", std::make_tuple(vertices_arg->get(), colliding_arg->get(), alpha_arg->get()));
    } else {
        ampprivate::pybridge::ScriptCaller::call("VisualizeLinkManipulator", "visualize_manipulator", std::make_tuple(vertices_arg->get(), colliding_arg->get()));
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

void amp::Visualizer::createAxes(const PotentialFunction2D& potential_function, const Problem2D& prob, std::size_t n_grid, bool vector, double u_min, double u_max) {
    // Bounds
    std::unique_ptr<ampprivate::pybridge::PythonObject> bounds_arg = workspaceBoundsToPythonObject(prob.x_min, prob.x_max, prob.y_min, prob.y_max);

    // Grid cells
    std::unique_ptr<ampprivate::pybridge::PythonObject> n_grid_arg = ampprivate::pybridge::makeLong(n_grid);

    // U values for two components of the vector field
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> u_values;
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> u1_values;  // For the x component
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> u2_values;  // For the y component

    u_values.reserve(n_grid * n_grid);
    u1_values.reserve(n_grid * n_grid);
    u2_values.reserve(n_grid * n_grid);

    Eigen::Vector2d lower_left(prob.x_min, prob.y_min);
    Eigen::Vector2d upper_right(prob.x_max, prob.y_max);
    Eigen::Vector2d disc_diag = 1.0 / static_cast<double>(n_grid) * (upper_right - lower_left);

    for (std::size_t i = 0; i < n_grid; ++i) {
        for (std::size_t j = 0; j < n_grid; ++j) {
            Eigen::Vector2d coord = lower_left + Eigen::Vector2d(static_cast<double>(i) * disc_diag[0], static_cast<double>(j) * disc_diag[1]);
            double u = potential_function(coord);
            u = std::min(u, u_max); // Clamp below u_max
            u = std::max(u, u_min); // Clamp above u_min
            u_values.push_back(ampprivate::pybridge::makeScalar(u));
            Eigen::Vector2d v = potential_function.getGradient(coord);
            u1_values.push_back(ampprivate::pybridge::makeScalar(v[0]));  // Store the x component
            u2_values.push_back(ampprivate::pybridge::makeScalar(v[1]));  // Store the y component
        }
    }

    std::unique_ptr<ampprivate::pybridge::PythonObject> u_values_arg = ampprivate::pybridge::makeList(std::move(u_values));
    std::unique_ptr<ampprivate::pybridge::PythonObject> u1_values_arg = ampprivate::pybridge::makeList(std::move(u1_values));
    std::unique_ptr<ampprivate::pybridge::PythonObject> u2_values_arg = ampprivate::pybridge::makeList(std::move(u2_values));

    // Call Python function to visualize
    if (vector)
        ampprivate::pybridge::ScriptCaller::call("VisualizePotentialFunction", "visualize_vector_field", std::make_tuple(bounds_arg->get(), n_grid_arg->get(), u1_values_arg->get(), u2_values_arg->get()));
    else
        ampprivate::pybridge::ScriptCaller::call("VisualizePotentialFunction", "visualize_potential_function", std::make_tuple(bounds_arg->get(), n_grid_arg->get(), u_values_arg->get()));
}

void amp::Visualizer::makeBoxPlot(const std::list<std::vector<double>>& data_sets, const std::vector<std::string>& labels, const std::string& title, const std::string& xlabel, const std::string& ylabel) {
    ASSERT(labels.size() == data_sets.size(), "Number of labels does not match number of data sets");
    newFigure();

    // Data
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> data_py_objects;
    data_py_objects.reserve(data_sets.size());
    for (const std::vector<double>& data_set : data_sets) {
        std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> data_points_py_objects;
        data_points_py_objects.reserve(data_set.size());
        for (double data_point : data_set) {
            data_points_py_objects.push_back(ampprivate::pybridge::makeScalar(data_point));
        }
        data_py_objects.push_back(ampprivate::pybridge::makeList(std::move(data_points_py_objects)));
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> data_arg = ampprivate::pybridge::makeList(std::move(data_py_objects));

    // Labels
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> labels_py_objects;
    labels_py_objects.reserve(labels.size());
    for (const std::string& label : labels) {
        labels_py_objects.push_back(ampprivate::pybridge::makeString(label));
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> labels_arg = ampprivate::pybridge::makeList(std::move(labels_py_objects));

    // Title
    std::unique_ptr<ampprivate::pybridge::PythonObject> title_arg = ampprivate::pybridge::makeString(title);

    // X-label
    std::unique_ptr<ampprivate::pybridge::PythonObject> xlabel_arg = ampprivate::pybridge::makeString(xlabel);

    // Y-label
    std::unique_ptr<ampprivate::pybridge::PythonObject> ylabel_arg = ampprivate::pybridge::makeString(ylabel);

    ampprivate::pybridge::ScriptCaller::call("Plotter", "make_box_plot", std::make_tuple(data_arg->get(), labels_arg->get(), title_arg->get(), xlabel_arg->get(), ylabel_arg->get()));
}

void amp::Visualizer::makeBarGraph(const std::vector<double>& values, const std::vector<std::string>& labels, const std::string& title, const std::string& xlabel, const std::string& ylabel) {
    ASSERT(labels.size() == values.size(), "Number of labels does not match number of values");
    newFigure();

    // Values
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> values_py_objects;
    values_py_objects.reserve(values.size());
    for (double value : values) {
        values_py_objects.push_back(ampprivate::pybridge::makeScalar(value));
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> values_arg = ampprivate::pybridge::makeList(std::move(values_py_objects));

    // Labels
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> labels_py_objects;
    labels_py_objects.reserve(labels.size());
    for (const std::string& label : labels) {
        labels_py_objects.push_back(ampprivate::pybridge::makeString(label));
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> labels_arg = ampprivate::pybridge::makeList(std::move(labels_py_objects));

    // Title
    std::unique_ptr<ampprivate::pybridge::PythonObject> title_arg = ampprivate::pybridge::makeString(title);

    // X-label
    std::unique_ptr<ampprivate::pybridge::PythonObject> xlabel_arg = ampprivate::pybridge::makeString(xlabel);

    // Y-label
    std::unique_ptr<ampprivate::pybridge::PythonObject> ylabel_arg = ampprivate::pybridge::makeString(ylabel);

    ampprivate::pybridge::ScriptCaller::call("Plotter", "make_bar_graph", std::make_tuple(values_arg->get(), labels_arg->get(), title_arg->get(), xlabel_arg->get(), ylabel_arg->get()));
}

#endif