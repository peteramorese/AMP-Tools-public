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
// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>
// #include <CGAL/Constrained_Delaunay_triangulation_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Projection_traits_xy_3.h>
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
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> bgPoint;



typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
// typedef CGAL::Exact_predicates_exact_constructions_kernel         EK;
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>            Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CDT::Face_handle                                          Face_handle;
typedef CDT::Point                                                Point;
typedef CGAL::Polygon_2<K>                                        Polygon_2;

typedef CGAL::Triangulation_vertex_base_with_info_3<unsigned, K>  Vb3;
typedef CGAL::Delaunay_triangulation_cell_base_3<K>                 Cb;
typedef CGAL::Triangulation_data_structure_3<Vb3, Cb>                TDS3;
typedef CGAL::Delaunay_triangulation_3<K, TDS3>                      CDT_3;
typedef CDT_3::Cell_handle                                          Cell_handle;
typedef CDT_3::Point                                              Point_3;
// typedef CGAL::Polygon_3<K>                                        Polygon_3;
typedef K::Point_3                                                Point_3;
typedef CDT_3::Finite_facets_iterator                             FFI_3;

typedef CGAL::Polyhedron_3<K> Polyhedron;


// Structure to represent a face and its adjacent faces
struct abstractionNode {
    char observation;
    Face_handle faceHandle;
    std::vector<uint32_t> neighbors;
    std::vector<Face_handle> neighborsFaces;
    std::array<Eigen::Vector2d, 3> vertices;
    double area;
};

struct Node3D {
    char label;
    Cell_handle faceHandle;
    std::vector<uint32_t> neighbors;
    std::vector<Cell_handle> neighborCells;
    std::array<Eigen::Vector3d, 3> vertices;
    double volume;
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

json serializeNode3D(const Node3D& node, uint32_t index) {
    json j;
    j["index"] = index;
    j["label"] = node.label;
    // Serialize vertices
    for (int i = 0; i < 4; ++i) {
        j["vertices"].push_back({node.vertices[i][0], node.vertices[i][1]});
    }
    return j;
}


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

void insertRandomPoints(CDT& cdt, const std::vector<Eigen::Vector2d>& boxVertices, int numPoints) {
    double minX = std::min({boxVertices[0].x(), boxVertices[1].x(), boxVertices[2].x(), boxVertices[3].x()});
    double minY = std::min({boxVertices[0].y(), boxVertices[1].y(), boxVertices[2].y(), boxVertices[3].y()});
    double maxX = std::max({boxVertices[0].x(), boxVertices[1].x(), boxVertices[2].x(), boxVertices[3].x()});
    double maxY = std::max({boxVertices[0].y(), boxVertices[1].y(), boxVertices[2].y(), boxVertices[3].y()});

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distX(minX, maxX);
    std::uniform_real_distribution<double> distY(minY, maxY);

    // Sample two random points within the box
    for (int i = 1; i < numPoints; ++i)
        cdt.insert(Point(distX(gen), distY(gen)));
}

void insertGridPoints(CDT& cdt, const std::vector<Eigen::Vector2d>& boxVertices, int gridSize) {
    double minX = std::min({boxVertices[0].x(), boxVertices[1].x(), boxVertices[2].x(), boxVertices[3].x()});
    double minY = std::min({boxVertices[0].y(), boxVertices[1].y(), boxVertices[2].y(), boxVertices[3].y()});
    double maxX = std::max({boxVertices[0].x(), boxVertices[1].x(), boxVertices[2].x(), boxVertices[3].x()});
    double maxY = std::max({boxVertices[0].y(), boxVertices[1].y(), boxVertices[2].y(), boxVertices[3].y()});

    // Sample two random points within the box
    double stepsX = (maxX - minX) / gridSize;
    double stepsY = (maxY - minY) / gridSize;
    std::cout << "\nSX" << stepsX << " SY " << stepsY << std::endl;

    for (int i = 0; i <= stepsX; ++i) {
        for (int j = 0; j <= stepsY; ++j) {
            double x = minX + i * gridSize;
            double y = minY + j * gridSize;
            std::cout << "\nX" <<x << " Y " << y  << std::endl;
            cdt.insert(Point(x, y));
        }
    }
}

// Function to calculate the centroid of a triangle defined by its vertices
Vector2d calculateTriangleCentroid(const std::vector<Eigen::Vector2d>& vertices) {
    double centroidX = (vertices[0].x() + vertices[1].x() + vertices[2].x()) / 3.0;
    double centroidY = (vertices[0].y() + vertices[1].y() + vertices[2].y()) / 3.0;
    return Vector2d(centroidX, centroidY);
}

std::unordered_map<uint32_t, Node3D> triangulate3D(const std::vector<Eigen::Vector3d>& workspace, const std::vector<std::pair<std::vector<Eigen::Vector3d>, char>>& polygons, int size) {
    // Create a constrained Delaunay triangulation
    CDT_3 cdt_3;
    
    std::vector<Point_3> poly;
    for (const auto& vertex : workspace)
        cdt_3.insert(Point_3(vertex[0], vertex[1], vertex[2]));
    
    for (const auto& polygon : polygons) {
        for (const auto& vertex : polygon.first)
            cdt_3.insert(Point_3(vertex[0], vertex[1], vertex[2]));
    }

    // Compute constrained Delaunay triangulation
    cdt_3.is_valid();

    //Mark facets that are inside the domain bounded by the polygon
    std::unordered_map<Face_handle, char> labels;
    std::unordered_map<uint32_t, Node3D> graph;
    std::vector<pair<uint32_t, Face_handle>> neigborPairs;
    std::unordered_map<Face_handle, uint32_t> faceIndices;
    uint32_t ri = 0;
    char label;

    // for (FFI_3 f = cdt_3.finite_cells_begin(); f != cdt_3.finite_cells_end(); ++f) {
    //     Node3D node;
    //     node.faceHandle = f;
    //     // Store vertices of the face
    //     for (int i = 0; i < 4; ++i) {
    //         Point_3 p = f->vertex(i)->point();
    //         node.vertices[i] = Eigen::Vector3d(p.x(), p.y(), p.z());
    //     }
    //     // Calculate centroid and check label
    //     // Eigen::Vector3d centroid = calculateTriangleCentroid(node.vertices);
    //     // Point_3 centroid_point(centroid[0], centroid[1], centroid[2]);
    //     // for (const auto& region : regions) {
    //     //     if (boost::geometry::within(centroid_point, region.first)) {
    //     //         node.label = region.second;
    //     //         break;
    //     //     } 
    //     //     node.label = 'e';
    //     // }
    //     // Store neighbor faces
    //     for (int i = 0; i < 3; ++i) {
    //         Face_handle_3 neighborFace = f->neighbor(i);
    //         if (cdt_3.is_infinite(neighborFace)) continue; // Skip infinite faces
    //         node.neighborsFaces.push_back(neighborFace);
    //     }
    //     // node.volume = calculateVolume(node.vertices);
    //     graph[ri++] = node; // Add node to the graph with index ri
    // }

    uint32_t nodeIndex = 0;
    for (auto f = cdt_3.finite_cells_begin(); f != cdt_3.finite_cells_end(); ++f) {
        Node3D node;
        node.faceHandle = f;
        for (int i = 0; i < 4; ++i) {
            Cell_handle neighborFace = f->neighbor(i);
            if (cdt_3.is_infinite(neighborFace)) continue; // Skip infinite faces
            node.neighborCells.push_back(neighborFace);
        }
        for (int i = 0; i < 4; ++i) {
            Point_3 p = f->vertex(i)->point();
            node.vertices[i] = Eigen::Vector3d(p.x(), p.y(), p.z());
        }
        graph[nodeIndex++] = node;
    }

    // for (uint32_t r_i = 0; r_i < graph.size(); ++r_i) {
    //     uint32_t ind = 0;
    //     for (auto nodeP : graph) {
    //         uint32_t r_p = nodeP.first;
    //         if (r_i == r_p) continue;
    //         for (Face_handle n_i : graph[r_i].neighborsFaces) {
    //             if (n_i == nodeP.second.faceHandle) {
    //                 graph[r_i].neighbors.push_back(r_p);
    //                 ind++;
    //             }
    //         }
    //         if (ind == graph[r_i].neighborsFaces.size()) break;
    //     }
    // }

    json jNodes; 
    for (const auto& entry : graph) {
        uint32_t index = entry.first;
        const Node3D& node = entry.second;
        // Serialize node and add to JSON array
        jNodes.push_back(serializeNode3D(node, index));
    }
    std::ofstream outFile("triangles.json");
    if (outFile.is_open()) {
        outFile << jNodes.dump(4); // Write pretty-printed JSON with 4 spaces indentation
        outFile.close();
        std::cout << "Triangles saved to triangles.json" << std::endl;
    }
    std::cout << "There are " << ri << " faces in the domain." << std::endl;
    std::cout << "ELEMENTS IN GRAPH: "<< graph.size() << std::endl;
    return graph;
}


std::unordered_map<uint32_t, abstractionNode> triangulatePolygon(const std::vector<Eigen::Vector2d>& boundingBoxVertices, const std::vector<std::pair<std::vector<Eigen::Vector2d>, char>>& polygons, int size) {
    
    // Create a constrained Delaunay triangulation
    CDT cdt;
    
    Polygon_2 box;
    // Insert bounding box vertices as constraints
    for (const auto& vertex : boundingBoxVertices)
        box.push_back(Point(vertex[0], vertex[1]));
    cdt.insert_constraint(box.vertices_begin(), box.vertices_end(), true);
    // insertGridPoints(cdt, boundingBoxVertices, size);

    // Insert polygon vertices as holes
    std::vector<pair<boostPolygon, char>> regions;
    for (const auto& polygon : polygons) {
        Polygon_2 region;
        boostPolygon boostRegion;
        std::string points = "POLYGON((";
        char label = polygon.second;
        // if (label == 'o') continue;
        for (const auto& vertex : polygon.first){
            region.push_back(Point(vertex[0], vertex[1]));
            points += std::to_string(vertex[0]) + " " + std::to_string(vertex[1]) + ",";
        }
        points = points.substr(0, points.size() - 1);
        points += "))";
        boost::geometry::read_wkt(points, boostRegion);
        regions.push_back({boostRegion, label});
        cdt.insert_constraint(region.vertices_begin(), region.vertices_end(), true);
        // insertAdditionalPoints(cdt, region, 2.1);
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
        std::vector<Eigen::Vector2d> triangle;
        for (int i = 0; i < 3; ++i) {
            triangle.push_back({f->vertex(i)->point().x(), f->vertex(i)->point().y()});
        }
        Vector2d cent = calculateTriangleCentroid(triangle);
        point centriod(cent.x(), cent.y());
        for (auto& region : regions) {
            if (boost::geometry::within(centriod, region.first)) {
                std::cout << cent << " labeled " << region.second << std::endl;
                label = region.second;
                break;
            } 
            label = 'e';
        }
        graph[ri].observation = label;
        graph[ri].faceHandle = f;
        for (int i = 0; i < 3; ++i) {
            graph[ri].vertices[i] = Eigen::Vector2d(f->vertex(i)->point().x(), f->vertex(i)->point().y());
            CDT::Face_handle neighborFace = f->neighbor(i);
            if (cdt.is_infinite(neighborFace)) continue; // Skip infinite faces
            graph[ri].neighborsFaces.push_back(neighborFace);
        }
        graph[ri].area = triangleArea(graph[ri].vertices);
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
    json jNodes; 
    for (const auto& entry : graph) {
        uint32_t index = entry.first;
        const abstractionNode& node = entry.second;
        // Serialize node and add to JSON array
        jNodes.push_back(serializeNode(node, index));
    }
    std::ofstream outFile("triangles.json");
    if (outFile.is_open()) {
        outFile << jNodes.dump(4); // Write pretty-printed JSON with 4 spaces indentation
        outFile.close();
        std::cout << "Triangles saved to triangles.json" << std::endl;
    }
    std::cout << "There are " << ri << " faces in the domain." << std::endl;
    assert(ri > 0);
    assert(ri < cdt.number_of_faces());
    // CGAL::draw(cdt, in_domain);
    // std::unordered_map<uint32_t, abstractionNode> graph = buildGraph(cdt, labels);
    std::cout << "ELEMENTS IN GRAPH: "<< graph.size() << std::endl;
    return graph;
}
