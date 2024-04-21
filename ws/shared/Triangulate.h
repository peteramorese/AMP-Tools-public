#include <vector>
#include <Eigen/Dense>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/draw_triangulation_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/Polygon_2.h>
#include <iostream>
#include <unordered_map>
#include <boost/property_map/property_map.hpp>
typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>            Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
typedef CDT::Face_handle                                          Face_handle;
typedef CDT::Point                                                Point;
typedef CGAL::Polygon_2<K>                                        Polygon_2;

std::vector<std::array<Eigen::Vector2d, 3>> triangulatePolygon(const std::vector<Eigen::Vector2d>& boundingBoxVertices, const std::vector<std::vector<Eigen::Vector2d>>& polygons) {
    // Create a constrained Delaunay triangulation
    CDT cdt;
    
    Polygon_2 box;
    // Insert bounding box vertices as constraints
    for (const auto& vertex : boundingBoxVertices)
        box.push_back(Point(vertex[0], vertex[1]));
    cdt.insert_constraint(box.vertices_begin(), box.vertices_end(), true);

    // Insert polygon vertices as holes
    for (const auto& polygon : polygons) {
        Polygon_2 region;
        for (const auto& vertex : polygon)
            region.push_back(Point(vertex[0], vertex[1]));
        cdt.insert_constraint(region.vertices_begin(), region.vertices_end(), true);
    }

    // Compute constrained Delaunay triangulation
    cdt.is_valid();
    std::unordered_map<Face_handle, bool> in_domain_map;
    boost::associative_property_map< std::unordered_map<Face_handle,bool> > in_domain(in_domain_map);

    //Mark facets that are inside the domain bounded by the polygon
    CGAL::mark_domain_in_triangulation(cdt, in_domain);
    unsigned int count=0;
    for (Face_handle f : cdt.finite_face_handles())
    {
        if ( get(in_domain, f) ) ++count;
    }
    std::cout << "There are " << count << " faces in the domain." << std::endl;
    assert(count > 0);
    assert(count < cdt.number_of_faces());
    CGAL::draw(cdt, in_domain);

    // Extract triangles from the triangulation
    std::vector<std::array<Eigen::Vector2d, 3>> triangleVertices;
    for (auto face_iter = cdt.finite_faces_begin(); face_iter != cdt.finite_faces_end(); ++face_iter) {
        std::array<Eigen::Vector2d, 3> triangle;
        for (int i = 0; i < 3; ++i) {
            auto vertex = face_iter->vertex(i)->point();
            triangle[i] = Eigen::Vector2d(vertex.x(), vertex.y());
        }
        triangleVertices.push_back(triangle);
    }

    return triangleVertices;
}

// #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Constrained_Delaunay_triangulation_2.h>
// #include <CGAL/draw_triangulation_2.h>
// #include <CGAL/mark_domain_in_triangulation.h>
// #include <CGAL/Polygon_2.h>
// #include <iostream>
// #include <unordered_map>
// #include <boost/property_map/property_map.hpp>
// typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
// typedef CGAL::Triangulation_vertex_base_2<K>                      Vb;
// typedef CGAL::Constrained_triangulation_face_base_2<K>            Fb;
// typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
// typedef CGAL::Exact_predicates_tag                                Itag;
// typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;
// typedef CDT::Face_handle                                          Face_handle;
// typedef CDT::Point                                                Point;
// typedef CGAL::Polygon_2<K>                                        Polygon_2;
// int triangle( )
// {
//   //construct two non-intersecting nested polygons
//   Polygon_2 polygon1;
//   polygon1.push_back(Point(0,0));
//   polygon1.push_back(Point(2,0));
//   polygon1.push_back(Point(2,2));
//   polygon1.push_back(Point(1,1.75));
//   polygon1.push_back(Point(0,2));
//   Polygon_2 polygon2;
//   polygon2.push_back(Point(0.5,0.5));
//   polygon2.push_back(Point(1.5,0.5));
//   polygon2.push_back(Point(1.5,1.5));
//   polygon2.push_back(Point(0.5,1.5));
//   //Insert the polygons into a constrained triangulation
//   CDT cdt;
//   cdt.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(), true);
//   cdt.insert_constraint(polygon2.vertices_begin(), polygon2.vertices_end(), true);
//   cdt.insert_constraint(Point(0.25, 0.25), Point(0.25, 1.75));
//   std::unordered_map<Face_handle, bool> in_domain_map;
//   boost::associative_property_map< std::unordered_map<Face_handle,bool> >
//     in_domain(in_domain_map);
//   //Mark facets that are inside the domain bounded by the polygon
//   CGAL::mark_domain_in_triangulation(cdt, in_domain);
//   unsigned int count=0;
//   for (Face_handle f : cdt.finite_face_handles())
//   {
//     if ( get(in_domain, f) ) ++count;
//   }
//   std::cout << "There are " << count << " faces in the domain." << std::endl;
//   assert(count > 0);
//   assert(count < cdt.number_of_faces());
//   CGAL::draw(cdt, in_domain);
//   return 0;
// }