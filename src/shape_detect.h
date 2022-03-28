//
// Created by txt on 2021/12/5.
//

#ifndef CLOUDMESH_MAIN_H
#define CLOUDMESH_MAIN_H

// STL includes.
#include <string>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
// CGAL includes.
#include <CGAL/memory.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Iterator_range.h>
#include <CGAL/HalfedgeDS_vector.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/Polyhedron_iostream.h>
//#include <CGAL/draw_polyhedron.h>
#include <CGAL/Simple_cartesian.h>
//#include <CGAL/draw_surface_mesh.h>


#include "vector"

// Type declarations.
using EPICK = CGAL::Exact_predicates_inexact_constructions_kernel;
using FT      = typename EPICK::FT;
using Point_3 = typename EPICK::Point_3;
using SMesh   = CGAL::Surface_mesh<Point_3>;

using Face_range   = typename SMesh::Face_range;
using Face_index = typename SMesh::Face_index;
using Color = CGAL::IO::Color;
typedef boost::graph_traits<SMesh>::face_descriptor face_descriptor;

// Choose the type of a container for a polygon mesh.
#define USE_SURFACE_MESH
#if defined(USE_SURFACE_MESH)
typedef CGAL::Polyhedron_3<EPICK> Polyhedron;
using Neighbor_query = CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<SMesh>;
using Region_type    = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<EPICK, SMesh>;
using Sorting        = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<EPICK, SMesh, Neighbor_query>;
#else
using Polygon_mesh = CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_3, CGAL::HalfedgeDS_vector>;
    using Face_range   = typename CGAL::Iterator_range<typename boost::graph_traits<Polygon_mesh>::face_iterator>;
    using Neighbor_query = CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<Polygon_mesh, Face_range>;
    using Region_type    = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<Kernel, Polygon_mesh, Face_range>;
    using Sorting        = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<Kernel, Polygon_mesh, Neighbor_query, Face_range>;
#endif
using Region  = std::vector<std::size_t>;
using Regions = std::vector<Region>;
using Vertex_to_point_map = typename Region_type::Vertex_to_point_map;
using Region_growing = CGAL::Shape_detection::Region_growing<Face_range, Neighbor_query, Region_type, typename Sorting::Seed_map>;

#endif //CLOUDMESH_MAIN_H
using namespace std;
