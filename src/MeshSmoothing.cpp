//
// Created by txt on 2021/12/10.
//

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>
#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

#include <CGAL/draw_surface_mesh.h>

#include <iostream>
#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel                 EPICK;
typedef CGAL::Surface_mesh<EPICK::Point_3>                                  SMesh;
typedef CGAL::dynamic_vertex_property_t<bool>                               Vertex_bool_property;
typedef typename boost::property_map<SMesh, Vertex_bool_property>::type     VCMap;
typedef boost::graph_traits<SMesh>::vertex_descriptor                       vertex_descriptor;
namespace PMP = CGAL::Polygon_mesh_processing;

void ShapeSmooth(SMesh& mesh, int nb_iterations, double time)
{
    std::set<SMesh::Vertex_index> constrained_vertices;
    for(SMesh::Vertex_index v : vertices(mesh))
    {
        if(is_border(v, mesh))
            constrained_vertices.insert(v);
    }

    std::cout << "Constraining: " << constrained_vertices.size() << " border vertices" << std::endl;
    CGAL::Boolean_property_map<std::set<SMesh::Vertex_index> > vcmap(constrained_vertices);

    std::cout << "Smoothing shape... (" << nb_iterations << " iterations)" << std::endl;
    PMP::smooth_shape(mesh, time, PMP::parameters::number_of_iterations(nb_iterations)
            .vertex_is_constrained_map(vcmap));
}

void MeshSmooth(SMesh& mesh, int nb_iterations)
{
    const unsigned int nb_iter = nb_iterations;
    const bool projection = true;
    const bool use_safety_measures = false;
    const bool constrain_border_vertices = false;
    const bool use_angle_smoothing = true;
    const bool use_area_smoothing = true;
    const bool use_Delaunay_flips = true;

    VCMap vcmap = get(Vertex_bool_property(), mesh);
    for(vertex_descriptor v : vertices(mesh))
        put(vcmap, v, false);

    PMP::smooth_mesh(mesh, PMP::parameters::do_project(projection)
                                            .number_of_iterations(nb_iter)
                                            .vertex_is_constrained_map(vcmap)
                                            .use_safety_constraints(use_safety_measures)
                                            .use_angle_smoothing(use_angle_smoothing)
                                            .use_area_smoothing(use_area_smoothing)
                                            .use_Delaunay_flips(use_Delaunay_flips));

}

int main(int argc, char* argv[])
{
    if (argc<=1)
    {
        std::cout<<"----- input sample"<<std::endl;
        std::cout<<"----- -- exe path/input.off 2 1 0.00001"<<std::endl;
        std::cout<<"----- -- exe - path - mesh_iter - shape_iter - shape_time  "<<std::endl<<std::endl;
        return EXIT_FAILURE;
    }

    const char* filename = (argc > 1) ? argv[1] : "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/reconstruction.off";

    SMesh mesh;
    //  读入的方式很关键
    if(!PMP::IO::read_polygon_mesh(filename, mesh))
    {
        std::cerr << "Invalid input." << std::endl;
        return 1;
    }

    const unsigned int mesh_iterations = (argc > 2) ? std::atoi(argv[2]) : 3; // 迭代次数
    const unsigned int shape_iterations = (argc > 3) ? std::atoi(argv[3]) : 1; // 迭代次数
    const double time = (argc > 4) ? std::atof(argv[4]) : 0.00001;  // 迭代时间
    CGAL::draw(mesh);
    MeshSmooth(mesh, int(ceil(mesh_iterations/2)));
    ShapeSmooth(mesh, shape_iterations, time);
    MeshSmooth(mesh, int(ceil(mesh_iterations/2)));

    CGAL::IO::write_polygon_mesh("/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/mesh_smoothed.off", mesh, CGAL::parameters::stream_precision(17));

    CGAL::draw(mesh);
    std::cout << "Done!" << std::endl;
    return EXIT_SUCCESS;
}