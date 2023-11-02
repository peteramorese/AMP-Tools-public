#pragma once

#include "tools/Visualizer.h"

#include "public/PythonObjects.h"
#include "public/ScriptCaller.h"

template <typename FXN>
void amp::Visualizer::makeFigure(const Problem2D& prob, const Graph<double>& coordinate_map, const FXN& getCoordinateFromNode) {
    createAxes(prob);
    createAxes(coordinate_map, getCoordinateFromNode);
}

template <typename FXN>
void amp::Visualizer::makeFigure(const Problem2D& prob, const Path2D& path, const Graph<double>& coordinate_map, const FXN& getCoordinateFromNode) {
    createAxes(prob);
    createAxes(coordinate_map, getCoordinateFromNode);
    createAxes(path);
}

template <typename FXN>
void amp::Visualizer::createAxes(const Graph<double>& map, const FXN& getCoordinateFromNode) {
    std::vector<amp::Node> nodes = map.nodes();
    
    // Nodes
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> nodes_py_objects;
    nodes_py_objects.reserve(nodes.size());
    for (amp::Node node : nodes) {
        nodes_py_objects.push_back(ampprivate::pybridge::makeLong(node));
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> nodes_arg = ampprivate::pybridge::makeList(std::move(nodes_py_objects));

    // Neighbors
    std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> connections_py_objects;
    connections_py_objects.reserve(nodes.size());
    for (amp::Node node : nodes) {
        const std::vector<amp::Node>& neighbors = map.children(node);
        std::vector<std::unique_ptr<ampprivate::pybridge::PythonObject>> neighbors_py_objects;
        neighbors_py_objects.reserve(neighbors.size());
        for (amp::Node neighbor : neighbors) {
            neighbors_py_objects.push_back(ampprivate::pybridge::makeLong(neighbor));
        }
        connections_py_objects.push_back(ampprivate::pybridge::makeList(std::move(neighbors_py_objects)));
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> connections_arg = ampprivate::pybridge::makeList(std::move(connections_py_objects));

    // Coordinates
    ampprivate::pybridge::ListOfPairs<double> coords;
    coords.list_of_tuples.resize(nodes.size());
    auto it = coords.list_of_tuples.begin();
    for (amp::Node node : nodes) {
        auto coord = getCoordinateFromNode(node);
        (*it)[0] = coord[0];
        (*it)[1] = coord[1];
        ++it;
    }
    std::unique_ptr<ampprivate::pybridge::PythonObject> coords_arg = coords.toPyList();

    ampprivate::pybridge::ScriptCaller::call("VisualizeCoordinateMap", "visualize_coordinate_map", std::make_tuple(nodes_arg->get(), connections_arg->get(), coords_arg->get()));
}