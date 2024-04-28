#pragma once
#include <vector>
#include <Eigen/Dense>
#include "HelpfulClass.h"
#include <fstream>
#include "json.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/Polygon_2.h>
#include <iostream>
#include <unordered_map>
#include <boost/geometry.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

using std::vector, std::string, std::cout, Eigen::Vector2d;
using json = nlohmann::json;
typedef boost::geometry::model::polygon<point> boostPolygon; 

typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>            Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CDT::Face_handle                                          Face_handle;
typedef CDT::Point                                                Point;
typedef CGAL::Polygon_2<K>                                        Polygon_2;


// Structure to represent a face and its adjacent faces
struct abstractionNode {
    char observation;
    Face_handle faceHandle;
    std::vector<uint32_t> neighbors;
    std::vector<Face_handle> neighborsFaces;
    std::array<Eigen::Vector2d, 3> vertices;
};

json serializeNode(const abstractionNode& node, uint32_t index) {
    json j;
    j["index"] = index;
    j["observation"] = node.observation;
    // Serialize vertices
    for (int i = 0; i < 3; ++i) {
        j["vertices"].push_back({node.vertices[i][0], node.vertices[i][1]});
    }
    return j;
}

// class Abstraction {
// public:
//     Abstraction(const std::vector<Eigen::Vector2d>& boundingBoxVertices, const std::vector<std::pair<std::vector<Eigen::Vector2d>, char>>& polygons) {
//         // Create a constrained Delaunay triangulation
//         // buildTriangulation(boundingBoxVertices, polygons);
//     }

//     const std::unordered_map<int, xState>& getGraph() const {
//         return graph;
//     }

// private:
//     std::unordered_map<int, xState> graph;

//     void buildGraphd(const CDT& cdt, const std::unordered_map<Face_handle, char>& labels) {
//         std::unordered_map<Face_handle, int> faceIndices;
//         std::unordered_map<int, xState> graph;

//         int index = 0;
//         for (auto i = cdt.finite_faces_begin(); i != cdt.finite_faces_end(); ++i, ++index) 
//             faceIndices[i] = index;
        
//         for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); ++face) {
//             if (cdt.is_infinite(face)) continue; // Skip infinite faces
//             int faceIndex = faceIndices[face];
//             for (int i = 0; i < 3; ++i) {
//                 // graph[faceIndex].vertices[i] = Eigen::Vector2d(face->vertex(i)->point().x(), face->vertex(i)->point().y());
//                 CDT::Face_handle neighborFace = face->neighbor(i);
//                 if (cdt.is_infinite(neighborFace)) continue; // Skip infinite faces

//                 int neighborIndex = faceIndices[neighborFace];
//                 graph[faceIndex].neighbors.push_back(neighborIndex);
//                 graph[faceIndex].weights.push_back(1.0);
//             }
//             graph[faceIndex].observation = labels.at(face);
//             // std::cout << "Observation: " << labels.at(face) << std::endl;
//         }
//     }
// }


// std::unordered_map<uint32_t, abstractionNode> buildGraph(const CDT& cdt, const std::unordered_map<Face_handle, char>& labels) {
//     std::unordered_map<Face_handle, int> faceIndices;
//     std::unordered_map<uint32_t, abstractionNode> graph;

//     uint32_t index = 0;
//     for (auto i = cdt.finite_faces_begin(); i != cdt.finite_faces_end(); ++i, ++index) 
//         faceIndices[i] = index;
    
//     for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); ++face) {
//         if (cdt.is_infinite(face)) continue; // Skip infinite faces
//         uint32_t faceIndex = faceIndices[face];
//         for (int i = 0; i < 3; ++i) {
//             graph[faceIndex].vertices[i] = Eigen::Vector2d(face->vertex(i)->point().x(), face->vertex(i)->point().y());
//             CDT::Face_handle neighborFace = face->neighbor(i);
//             if (cdt.is_infinite(neighborFace)) continue; // Skip infinite faces

//             uint32_t neighborIndex = faceIndices[neighborFace];
//             graph[faceIndex].neighbors.push_back(neighborIndex);
//             // graph[faceIndex].weights.push_back(1.0);
//         }
//         graph[faceIndex].observation = labels.at(face);
//         // std::cout << "Observation: " << labels.at(face) << std::endl;
//     }

//     // for (const auto& [faceIndex, faceInfo] : graph) {
//     //     std::cout << "Face " << faceIndex << " is adjacent to faces with weights initialized to 1: ";
//     //     for (size_t i = 0; i < faceInfo.neighbors.size(); ++i) std::cout << faceInfo.neighbors[i] << " ";
//     //     std::cout << std::endl;
//     // }

//     return graph;
// }

void insertAdditionalPoints(CDT& cdt, const Polygon_2& polygon, double maxEdgeLength) {
    for (auto edge = polygon.edges_begin(); edge != polygon.edges_end(); ++edge) {
        auto source = edge->source();
        auto target = edge->target();
        double dx = target.x() - source.x();
        double dy = target.y() - source.y();
        double length = std::sqrt(dx * dx + dy * dy);
        int numPoints = std::ceil(length / maxEdgeLength);

        for (int i = 1; i < numPoints; ++i) {
            double x = source.x() + i * dx / numPoints;
            double y = source.y() + i * dy / numPoints;
            cdt.insert(Point(x, y));
        }
    }
}

std::unordered_map<uint32_t, abstractionNode> triangulatePolygon(const std::vector<Eigen::Vector2d>& boundingBoxVertices, const std::vector<std::pair<std::vector<Eigen::Vector2d>, char>>& polygons) {
    // Create a constrained Delaunay triangulation
    CDT cdt;
    
    Polygon_2 box;
    // Insert bounding box vertices as constraints
    for (const auto& vertex : boundingBoxVertices)
        box.push_back(Point(vertex[0], vertex[1]));
    cdt.insert_constraint(box.vertices_begin(), box.vertices_end(), true);
    insertAdditionalPoints(cdt, box, 2.1);

    // Insert polygon vertices as holes
    std::vector<pair<boostPolygon, char>> regions;
    for (const auto& polygon : polygons) {
        Polygon_2 region;
        boostPolygon boostRegion;
        std::string points = "POLYGON((";
        char label = polygon.second;
        for (const auto& vertex : polygon.first){
            region.push_back(Point(vertex[0], vertex[1]));
            points += std::to_string(vertex[0]) + " " + std::to_string(vertex[1]) + ",";
        }
        points = points.substr(0, points.size() - 1);
        points += "))";
        boost::geometry::read_wkt(points, boostRegion);
        regions.push_back({boostRegion, label});
        cdt.insert_constraint(region.vertices_begin(), region.vertices_end(), true);
        insertAdditionalPoints(cdt, region, 2.1);
    }

    // Compute constrained Delaunay triangulation
    cdt.is_valid();
    std::unordered_map<Face_handle, bool> in_domain_map;
    boost::associative_property_map< std::unordered_map<Face_handle,bool> > in_domain(in_domain_map);

    //Mark facets that are inside the domain bounded by the polygon
    CGAL::mark_domain_in_triangulation(cdt, in_domain);
    std::unordered_map<Face_handle, char> labels;
    std::unordered_map<uint32_t, abstractionNode> graph;
    std::vector<pair<uint32_t, Face_handle>> neigborPairs;
    std::unordered_map<Face_handle, uint32_t> faceIndices;
    unsigned int ri=0;
    char label;
    for (Face_handle f : cdt.finite_face_handles()) {
        if ( get(in_domain, f) ) {
            std::string points = "POLYGON((";
            for (int i = 0; i < 3; ++i) {
                points += std::to_string(f->vertex(i)->point().x()) + " " + std::to_string(f->vertex(i)->point().y());
                if (i == 2) points += "))";
                else points += ",";
            }
            std::cout << points << " e\n";
            label = 'e';
        } else {
            boostPolygon face;
    		std::string points = "POLYGON((";
            for (int i = 0; i < 3; ++i) {
                points += std::to_string(f->vertex(i)->point().x()) + " " + std::to_string(f->vertex(i)->point().y());
                if (i == 2) points += "))";
                else points += ",";
            }
            boost::geometry::read_wkt(points, face);
            for (auto region: regions) {
                if (!boost::geometry::disjoint(region.first, face)) {
                    std::cout << points << " " << region.second <<std::endl;
                    label = region.second;
                    break;                    
                } 
            }
        }
        if (label == 'o') continue;
        graph[ri].observation = label;
        graph[ri].faceHandle = f;
        for (int i = 0; i < 3; ++i) {
            graph[ri].vertices[i] = Eigen::Vector2d(f->vertex(i)->point().x(), f->vertex(i)->point().y());
            CDT::Face_handle neighborFace = f->neighbor(i);
            if (cdt.is_infinite(neighborFace)) continue; // Skip infinite faces
            graph[ri].neighborsFaces.push_back(neighborFace);
            // neigborPairs.push_back({ri, neighborFace})
        }
        ri++;
    }
    for (uint32_t r_i = 0; r_i < graph.size(); ++r_i) {
        uint32_t ind = 0;
        for (auto nodeP : graph) {
            uint32_t r_p = nodeP.first;
            if (r_i == r_p) continue;
            for (Face_handle n_i : graph[r_i].neighborsFaces) {
                if (n_i == nodeP.second.faceHandle) {
                    graph[r_i].neighbors.push_back(r_p);
                    ind++;
                }
            }
            if (ind == graph[r_i].neighborsFaces.size()) break;
        }
    }
    // json jNodes; 
    // for (const auto& entry : graph) {
    //     uint32_t index = entry.first;
    //     const abstractionNode& node = entry.second;
    //     // Serialize node and add to JSON array
    //     jNodes.push_back(serializeNode(node, index));
    // }
    // std::ofstream outFile("triangles.json");
    // if (outFile.is_open()) {
    //     outFile << jNodes.dump(4); // Write pretty-printed JSON with 4 spaces indentation
    //     outFile.close();
    //     std::cout << "Triangles saved to triangles.json" << std::endl;
    // }
    std::cout << "There are " << ri << " faces in the domain." << std::endl;
    assert(ri > 0);
    assert(ri < cdt.number_of_faces());
    // CGAL::draw(cdt, in_domain);
    // std::unordered_map<uint32_t, abstractionNode> graph = buildGraph(cdt, labels);
    std::cout << "ELEMENTS IN GRAPH: "<< graph.size() << std::endl;
    return graph;
}
