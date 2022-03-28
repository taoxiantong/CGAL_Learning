//
// Created by txt on 2021/12/7.
//

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
#include <fstream>
typedef CGAL::Simple_cartesian<double>                       Kernel;
typedef Kernel::Point_3                                      Point;
typedef CGAL::Surface_mesh<Point>                            Mesh;
//   CGAL::Surface_mesh< CGAL::Simple_cartesian<double>::Point_3 >
int main(int argc, char* argv[])
{
    const char* filename = (argc>1) ? argv[1] : "../../data/cube.off";
    Mesh sm;
    if(!CGAL::IO::read_polygon_mesh(filename, sm))
    {
        std::cerr << "Invalid input file." << std::endl;
        return EXIT_FAILURE;
    }
    CGAL::draw(sm);
    return EXIT_SUCCESS;
}