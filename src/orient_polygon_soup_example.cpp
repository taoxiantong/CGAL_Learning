#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

#include <CGAL/IO/polygon_soup_io.h>

#include <vector>
#include <fstream>
#include <iostream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel          K;
typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3>      Polyhedron;

int main(int argc, char* argv[])
{
  const char* filename = (argc > 1) ? argv[1] : "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/2mesh.off";

  std::vector<K::Point_3> points;
  std::vector<std::vector<std::size_t> > polygons;

  if(!CGAL::IO::read_polygon_soup(filename, points, polygons) || points.empty())
  {
    std::cerr << "Cannot open file " << std::endl;
    return EXIT_FAILURE;
  }

  CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons);

  Polyhedron mesh;
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons, mesh);

  // Number the faces because 'orient_to_bound_a_volume' needs a face <--> index map
  int index = 0;
  for(Polyhedron::Face_iterator fb=mesh.facets_begin(), fe=mesh.facets_end(); fb!=fe; ++fb)
    fb->id() = index++;

  // if(CGAL::is_closed(mesh))
  //   CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(mesh);
  //   std::cout<<"here"<<std::endl;
  // std::ofstream out("/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/2mesh-oriented1.off", std::ios::out);
  // out.precision(17);
  // out << mesh;
  // out.close();

    std::cout<<"here"<<std::endl;

  CGAL::Polygon_mesh_processing::reverse_face_orientations(mesh);
  std::ofstream out2("/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/2mesh-oriented2.off", std::ios::out);
  out2.precision(17);
  out2 << mesh;
  out2.close();
  std::cout<<"here"<<std::endl;
  return EXIT_SUCCESS;
}
