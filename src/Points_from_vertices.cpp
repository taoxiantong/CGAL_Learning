//
// Created by txt on 2021/12/10.
//

#include "Points_from_vertices.h"

/*
Scene_points_with_normal_item_priv(const SMesh& input_mesh)
{
    m_points(new Point_set);

    boost::graph_traits<SMesh>::vertex_iterator v;
    m_points->add_normal_map();
    for (v = const_cast<SMesh&>(input_mesh).vertices_begin();  v != const_cast<SMesh&>(input_mesh).vertices_end(); v++)
    {
        boost::graph_traits<SMesh>::vertex_descriptor vd(*v);
        const Kernel::Point_3& p = input_mesh.point(vd);
        Kernel::Vector_3 n =
                CGAL::Polygon_mesh_processing::compute_vertex_normal(vd, input_mesh);
        m_points->insert(p,n);
    }
}
 */


int main(int argc, char* argv[])
{
    const char* filename = (argc > 1) ? argv[1] : "../../data/polygon_mesh.off";

    SMesh input_mesh;
    if(!PMP::IO::read_polygon_mesh(filename, input_mesh))
    {
        std::cerr << "Invalid input." << std::endl;
        return 1;
    }

    Point_set m_points;
//    m_points.add_normal_map();
    m_points.add_normal_map();
    boost::graph_traits<SMesh>::vertex_iterator v;
    for (v = const_cast<SMesh&>(input_mesh).vertices_begin();  v != const_cast<SMesh&>(input_mesh).vertices_end(); v++)
    {
        boost::graph_traits<SMesh>::vertex_descriptor vd(*v);
        const Kernel::Point_3& p = input_mesh.point(vd);
        Kernel::Vector_3 n =
                CGAL::Polygon_mesh_processing::compute_vertex_normal(vd, input_mesh);
        m_points.insert(p,n);
    }

//    std::ofstream f("../../normal_out_polygon_mesh.ply", std::ios::binary);
//    CGAL::IO::set_binary_mode(f); // The PLY file will be written in the binary format
//    CGAL::IO::write_PLY_with_properties(f, m_points,
//                                        CGAL::make_ply_point_writer (Point_map()),
//                                        std::make_tuple(Color_map(),
//                                                        CGAL::IO::PLY_property<unsigned char>("red"),
//                                                        CGAL::IO::PLY_property<unsigned char>("green"),
//                                                        CGAL::IO::PLY_property<unsigned char>("blue"),
//                                                        CGAL::IO::PLY_property<unsigned char>("alpha")),
//                                        std::make_pair(Intensity_map(), CGAL::IO::PLY_property<int>("intensity")));
    std::string output_filename("../../data/normal_out_polygon_mesh.ply");
//    if(!CGAL::IO::write_points(output_filename, m_points,
//                               CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
//                                       .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
//                                       .stream_precision(17)))
//    {
//        std::cerr << "Error: cannot write file " << output_filename << std::endl;
//        return EXIT_FAILURE;
//    }

    if (!CGAL::IO::write_PLY(output_filename, m_points))
    {
        std::cerr << "Error: cannot write file " << output_filename << std::endl;
        return EXIT_FAILURE;
    }

    /*
    m_points(new Point_set);
    boost::graph_traits<SMesh>::vertex_iterator v;
    m_points->add_normal_map();
    for (v = const_cast<SMesh&>(input_mesh).vertices_begin();  v != const_cast<SMesh&>(input_mesh).vertices_end(); v++)
    {
        boost::graph_traits<SMesh>::vertex_descriptor vd(*v);
        const Kernel::Point_3& p = input_mesh.point(vd);
        Kernel::Vector_3 n =
                CGAL::Polygon_mesh_processing::compute_vertex_normal(vd, input_mesh);
        m_points->insert(p,n);
    }
     */
    std::cout << "SUCCESS: write file " << output_filename << std::endl;
    return EXIT_SUCCESS;
}