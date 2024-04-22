#pragma once
#include <vector>
#include <Eigen/Dense>
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
struct xState {
    char observation;
    std::vector<uint32_t> neighbors;
    std::vector<double> weights;
    std::array<Eigen::Vector2d, 3> vertices;
};

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


std::unordered_map<uint32_t, xState> buildGraph(const CDT& cdt, const std::unordered_map<Face_handle, char>& labels) {
    std::unordered_map<Face_handle, int> faceIndices;
    std::unordered_map<uint32_t, xState> graph;

    uint32_t index = 0;
    for (auto i = cdt.finite_faces_begin(); i != cdt.finite_faces_end(); ++i, ++index) 
        faceIndices[i] = index;
    
    for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); ++face) {
        if (cdt.is_infinite(face)) continue; // Skip infinite faces
        uint32_t faceIndex = faceIndices[face];
        for (int i = 0; i < 3; ++i) {
            graph[faceIndex].vertices[i] = Eigen::Vector2d(face->vertex(i)->point().x(), face->vertex(i)->point().y());
            CDT::Face_handle neighborFace = face->neighbor(i);
            if (cdt.is_infinite(neighborFace)) continue; // Skip infinite faces

            uint32_t neighborIndex = faceIndices[neighborFace];
            graph[faceIndex].neighbors.push_back(neighborIndex);
            graph[faceIndex].weights.push_back(1.0);
        }
        graph[faceIndex].observation = labels.at(face);
        // std::cout << "Observation: " << labels.at(face) << std::endl;
    }

    // for (const auto& [faceIndex, faceInfo] : graph) {
    //     std::cout << "Face " << faceIndex << " is adjacent to faces with weights initialized to 1: ";
    //     for (size_t i = 0; i < faceInfo.neighbors.size(); ++i) std::cout << faceInfo.neighbors[i] << " ";
    //     std::cout << std::endl;
    // }

    return graph;
}

std::unordered_map<uint32_t, xState> triangulatePolygon(const std::vector<Eigen::Vector2d>& boundingBoxVertices, const std::vector<std::pair<std::vector<Eigen::Vector2d>, char>>& polygons) {
    // Create a constrained Delaunay triangulation
    CDT cdt;
    
    Polygon_2 box;
    // Insert bounding box vertices as constraints
    for (const auto& vertex : boundingBoxVertices)
        box.push_back(Point(vertex[0], vertex[1]));
    cdt.insert_constraint(box.vertices_begin(), box.vertices_end(), true);

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
    }

    // Compute constrained Delaunay triangulation
    cdt.is_valid();
    std::unordered_map<Face_handle, bool> in_domain_map;
    boost::associative_property_map< std::unordered_map<Face_handle,bool> > in_domain(in_domain_map);

    //Mark facets that are inside the domain bounded by the polygon
    CGAL::mark_domain_in_triangulation(cdt, in_domain);
    std::unordered_map<Face_handle, char> labels;
    unsigned int count=0;
    for (Face_handle f : cdt.finite_face_handles()) {
        if ( get(in_domain, f) ) {
            labels[f] = 'e';
            ++count;
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
                    labels[f] = region.second;
                    break;                    
                } 
            }
        }
    }
    std::cout << "There are " << count << " faces in the domain." << std::endl;
    assert(count > 0);
    assert(count < cdt.number_of_faces());
    // CGAL::draw(cdt, in_domain);
    std::unordered_map<uint32_t, xState> graph = buildGraph(cdt, labels);
    // std::cout << "WEIGHTTTTT "<< graph[0].weights[0] << std::endl;
    return graph;
}
