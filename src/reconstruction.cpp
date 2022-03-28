// #include "Kernel_type.h"
// #include "SMesh_type.h"
// #include "Scene_points_with_normal_item.h"
// #include "Scene_polygon_soup_item.h"
// #include "Point_set_3.h"
#include <CGAL/Surface_mesh.h>

#include <CGAL/compute_average_spacing.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Alpha_shape_mesher.h>
#include <CGAL/Scale_space_reconstruction_3/Weighted_PCA_smoother.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_3/Robust_intersection_traits_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Point_set_3/IO/PLY.h>
#include <CGAL/Timer.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/draw_surface_mesh.h>
// File_writer_OFF
#include <fstream>
#include <iostream>
#include <ostream>
#include <vector>



#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel     EPICK;
typedef CGAL::Mesh_3::Robust_intersection_traits_3< EPICK >     Kernel;
typedef CGAL::Point_set_3<Kernel::Point_3, Kernel::Vector_3>    Point_set;
typedef CGAL::Surface_mesh<EPICK::Point_3>                      SMesh;
// Concurrency

typedef CGAL::Scale_space_surface_reconstruction_3<Kernel> ScaleSpace;
typedef CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel> ScaleSpaceAFM;
typedef CGAL::Scale_space_reconstruction_3::Alpha_shape_mesher<Kernel> ScaleSpaceASM;
typedef CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel> ScaleSpaceJS;
typedef CGAL::Scale_space_reconstruction_3::Weighted_PCA_smoother<Kernel> ScaleSpaceWPS;

typedef EPICK::Point_3 Point_3;
typedef std::vector<Point_3> Points;

typedef std::vector<std::size_t> Polygon_3;
typedef std::vector<Polygon_3> Polygons;

// jet_smoother = true
// iterations = 4
// neighbors = 16 
// fitting = 2
// monge = 2
// advancing_front_mesher = false
// generate_smooth = false
//  longest_edge 0,  radius_ratio_bound 5,  beta_angle 30,

// separate_shells = fasle
// force_manifold = true

struct Polygon_soup
{
    Points points;
    Polygons polygons;
};



void scale_space (const Point_set& points, unsigned int iterations,
                  unsigned int neighbors, unsigned int fitting, unsigned int monge)
{
    // 定义新建立的数据
    Polygon_soup soup;
    ScaleSpace reconstruct (points.points().begin(), points.points().end());


    double squared_radius = 0.;
    //  jet_smoother = true

    // neighbors = 16
    // fitting = 2
    // monge = 2
    ScaleSpaceJS smoother(neighbors, fitting, monge);
    reconstruct.increase_scale(iterations, smoother);
    // if (!advancing_front_mesher)
    typedef std::pair<Kernel::Point_3, Kernel::Vector_3> PointVectorPair;
    squared_radius = 0.002;
    // squared_radius = CGAL::compute_average_spacing<CGAL::Parallel_if_available_tag> (points, 16); //CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())

    ScaleSpaceASM mesher (squared_radius, false, true);
    reconstruct.reconstruct_surface (mesher);

    // force_manifold = true

    std::ptrdiff_t num = std::distance( mesher.garbage_begin(  ),
                                                mesher.garbage_end(  ) );

            // Scene_polygon_soup_item* new_item = new Scene_polygon_soup_item ();
            // new_item->setColor(Qt::blue);
            // new_item->setRenderingMode(FlatPlusEdges);
            // new_item->init_polygon_soup(points.size(), num);
                // d->soup->points.reserve(nb_pts);
                // d->soup->polygons.reserve(nb_polygons);

    soup.points.reserve(points.size());
    soup.polygons.reserve(num);

    std::map<std::size_t, std::size_t> map_i2i;

    std::size_t current_index = 0;
    for (ScaleSpaceASM::Facet_iterator it=mesher.garbage_begin(), end=mesher.garbage_end();it!=end;++it)
    {
        for (unsigned int ind = 0; ind < 3; ++ ind)
        {
            if (map_i2i.find ((*it)[ind]) == map_i2i.end ())
            {
                map_i2i.insert (std::make_pair ((*it)[ind], current_index ++));
                Point_3 p = points.point(*(points.begin() + (*it)[ind]));       // 这句有问题
                        // Point_3 p = points.point(*(points.begin_or_selection_begin() + (*it)[ind]));
                            //   const_iterator begin_or_selection_begin() const
                            //   {
                            //     return (this->m_nb_removed == 0 ? begin() : first_selected());
                            //   }
                            //   iterator begin_or_selection_begin()
                            //   {
                            //     return (this->m_nb_removed == 0 ? begin() : first_selected());
                            //   }
                soup.points.push_back(Point_3(p.x (), p.y (), p.z ()));

            }
        }
                // new_item->new_triangle( map_i2i[(*it)[0]],
                //                         map_i2i[(*it)[1]],
                //                         map_i2i[(*it)[2]] );

        Polygon_3 new_polygon(3);
        new_polygon[0] = map_i2i[(*it)[0]];
        new_polygon[1] = map_i2i[(*it)[1]];
        new_polygon[2] = map_i2i[(*it)[2]];
        soup.polygons.push_back(new_polygon);

    }


    SMesh myMesh;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh< CGAL::Surface_mesh<Point_3> >(
                                    soup.points, soup.polygons, myMesh);
    std::size_t rv = CGAL::Polygon_mesh_processing::remove_isolated_vertices(myMesh);
    if(rv > 0){
        std::cerr << "Ignore isolated vertices: " << rv << std::endl;
        myMesh.collect_garbage();
    }
    if(myMesh.vertices().size() > 0) {
        std::cout << "【success!】 " <<std::endl;
    }

    std::string output_filename("/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/reconstruction.off");
    CGAL::IO::write_polygon_mesh(output_filename, myMesh, CGAL::parameters::stream_precision(17));
    CGAL::draw(myMesh);
}





// jet 模式
// Alpha Shape 模式
int main(int argc, char** argv)
{

    // jet_smoother = true
    // iterations = 4
    // neighbors = 16 
    // fitting = 2
    // monge = 2
    // advancing_front_mesher = false
    // generate_smooth = false
    //  longest_edge 0,  radius_ratio_bound 5,  beta_angle 30,

    // separate_shells = true
    // force_manifold = true

    const char* filename1 = (argc > 1) ? argv[1] : "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/point2.ply";
    std::cerr << "Reading " << std::flush;
    Point_set m_point_set;

    // 读入点云
    if (!(CGAL::IO::read_PLY(filename1, m_point_set)))
    {
        std::cerr << "【Error】: cannot read file" << std::endl;
        return EXIT_FAILURE;
    }
    std::cerr << "done: " << m_point_set.size() << " points." << std::endl;
    std::cout << "done: " << m_point_set.size() << " points." << std::endl;
    

    scale_space (m_point_set, 4, 16, 2, 2);
    return 0;
}
