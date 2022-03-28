#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_3/Robust_intersection_traits_3.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Point_set_3/IO/PLY.h>
#include <CGAL/Timer.h>
#include <CGAL/Point_set_3.h>
#include <fstream>
#include <iostream>

// #include <CGAL/IO/PLY.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel     EPICK;
typedef EPICK::Point_3                                          Point;
typedef CGAL::Scale_space_surface_reconstruction_3<EPICK>       Reconstruction;
typedef Reconstruction::Facet_const_iterator                    Facet_iterator;
typedef CGAL::Mesh_3::Robust_intersection_traits_3<EPICK>       Kernel;
typedef CGAL::Point_set_3<Kernel::Point_3, Kernel::Vector_3>    Point_set;

typedef CGAL::Scale_space_reconstruction_3::Weighted_PCA_smoother< EPICK >  Smoother;
typedef CGAL::Scale_space_reconstruction_3::Alpha_shape_mesher< EPICK >     Mesher;



using namespace std;

// function for writing the reconstruction output in the off format
void dump_reconstruction(const Reconstruction& reconstruct, std::string name)
{
  std::ofstream output(name.c_str());
  output << "OFF " << reconstruct.number_of_points() << " "
         << reconstruct.number_of_facets() << " 0\n";
  std::copy(reconstruct.points_begin(),
            reconstruct.points_end(),
            std::ostream_iterator<Point>(output,"\n"));
  for( Facet_iterator it = reconstruct.facets_begin(); it != reconstruct.facets_end(); ++it )
      output << "3 " << *it << std::endl;
}

int main(int argc, char** argv)
{
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

    std::cerr << "Reconstruction ";
    CGAL::Timer t;
    t.start();
    // 构造重建
    Reconstruction reconstruct (m_point_set.points().begin(), m_point_set.points().end());
    // 两次平滑
    // const int V1 = (argc > 2) ? atoi(argv[2]) : 10;
    // const int V2 = (argc > 3) ? atoi(argv[3]) : 100;
    // for (std::size_t i = 0; i < 2; ++ i)
    // {
    //     // Construct the smoother with parameters for
    //     // the neighborhood squared radius estimation.
    //     Smoother smoother( 10, 100 );
    //     // Advance the scale-space several steps.
    //     // This automatically estimates the scale-space.
    //     reconstruct.increase_scale( 2, smoother );
    //     // Reconstruct the surface from the current scale-space.
    //     std::cout << "Neighborhood squared radius is "
    //                 << smoother.squared_radius() << std::endl;
    //     Mesher mesher (smoother.squared_radius());
    //     reconstruct.reconstruct_surface(mesher);
    // }

    // dump_reconstruction(reconstruct, "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/reconstruction2.off");
    // std::cout << "Reconstructions are ready to be examinated in your favorite viewer" << std::endl;

    reconstruct.increase_scale(1);
    reconstruct.reconstruct_surface();

    std::cerr << "done in " << t.time() << " sec." << std::endl;
    t.reset();
    const char* filename2 = (argc > 2) ? argv[2] : "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/scale_space_out1.off";
    std::ofstream out (filename2);
    out << reconstruct;
    std::cerr << "Writing result in " << t.time() << " sec." << std::endl;
    std::cerr << "Done." << std::endl;
    return EXIT_SUCCESS;
}
