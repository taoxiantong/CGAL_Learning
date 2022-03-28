//
// Created by txt on 2021/12/10.
//

#ifndef CLOUDMESH_POINTS_FROM_VERTICES_H
#define CLOUDMESH_POINTS_FROM_VERTICES_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_3/Robust_intersection_traits_3.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

// Points includes
#include <CGAL/Point_set_3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

#include <CGAL/property_map.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/IO/write_points.h>
#include <CGAL/Point_set_3/IO/PLY.h>


#include <utility>

#include <iostream>
#include <fstream>


typedef CGAL::Exact_predicates_inexact_constructions_kernel   EPICK;
typedef CGAL::Surface_mesh<EPICK::Point_3>                    SMesh;
typedef EPICK::Point_3 Point;
typedef EPICK::Vector_3 Vector;
// Point with normal vector stored as a std::pair.
typedef std::pair<Point, Vector> Pwn;


typedef CGAL::Mesh_3::Robust_intersection_traits_3<EPICK> Kernel;
typedef CGAL::Point_set_3<Kernel::Point_3, Kernel::Vector_3> Point_set;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3> PointVectorPair;

namespace PMP = CGAL::Polygon_mesh_processing;


#endif //CLOUDMESH_POINTS_FROM_VERTICES_H
