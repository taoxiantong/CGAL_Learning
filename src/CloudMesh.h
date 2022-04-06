//
// Created by txt on 2021/12/5.
//

#ifndef CLOUDMESH_CLOUDMESH_H
#define CLOUDMESH_CLOUDMESH_H

// STL includes.
#include <iostream>
#include <fstream>
#include "vector"
#include <utility>
#include <iostream>
#include <iterator>
#include <string>
#include <cstdlib>
#include <cmath>
#include <math.h>

#include "plot.h"


// CGAL includes.
#include <CGAL/memory.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Iterator_range.h>
#include <CGAL/HalfedgeDS_vector.h>

#include <CGAL/Bbox_3.h>


#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Mesh_3/Robust_intersection_traits_3.h>


#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>


#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>



#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup_extension.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>
#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>


// Points includes
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO/PLY.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh/IO/OFF.h>

#include <CGAL/IO/OFF.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/IO/write_points.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/IO/polygon_soup_io.h>


#include <CGAL/draw_surface_mesh.h>
#include <CGAL/draw_point_set_3.h>

// ===========
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Alpha_shape_mesher.h>
#include <CGAL/Scale_space_reconstruction_3/Weighted_PCA_smoother.h>

#include <CGAL/Timer.h>


// PCL inlludes
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>


#include <Eigen/Core>

#include <boost/thread/thread.hpp>
#include <boost/format.hpp>


typedef CGAL::Exact_predicates_inexact_constructions_kernel   EPICK;
typedef CGAL::Surface_mesh<EPICK::Point_3>                    SMesh;



typedef EPICK::Point_3 Point_3;
typedef EPICK::Vector_3 Vector_3;
// Point with normal vector stored as a std::pair.
typedef std::pair<Point_3, Vector_3> Pwn;
// typedef std::array<unsigned char, 3> Color; 

typedef CGAL::Mesh_3::Robust_intersection_traits_3<EPICK>       Kernel;
typedef CGAL::Point_set_3<Kernel::Point_3, Kernel::Vector_3>    Point_set;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>            PointVectorPair;


// Type declarations.
using FT            =   typename EPICK::FT;
using Face_range    =   typename SMesh::Face_range;
using Face_index    =   typename SMesh::Face_index;
using Color         =   CGAL::IO::Color;
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

namespace PMP = CGAL::Polygon_mesh_processing;

using namespace std;
using PCLCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>;
using ShapeNormal = std::pair<pcl::PointXYZ, pcl::Normal>;
typedef CGAL::Bbox_3 Bbox;
#endif //CLOUDMESH_CLOUDMESH_H

class CloudMesh {
public:
    CloudMesh();
    ~CloudMesh();


    // 数据格式
    string FileInputPath;
    SMesh MeshInput;
    SMesh MeshShapeDetect;
    SMesh MeshOut;
    Point_set PointSetInput;
    vector<vector<uint32_t>> ShapePointsIndes;
    // Point_set m_points

    void draw();
    void draw(SMesh mesh);
    void draw(Point_set PointSet);

    // 0 打开文件
    bool OpenFile(string infile, SMesh& mesh);
    bool OpenFile(string infile, Point_set& MyPointSet);

    // 1. 平滑处理
    SMesh Smoothing(SMesh mesh, bool MeshS, const int Miter, bool ShapeS, const int Siter, const double time);

    // 2. 形状检测
    vector<vector<uint32_t>> ShapeDetect(SMesh mesh, double max_dis, int max_angle, int min_region_size);
    
    // 3. 从网格中提取点云，并计算法线
    Point_set PointSetFromVertices(SMesh mesh);
    
    // 4. 从网格中体形状部分的点云生成点云集
    Point_set SubMeshCreatFromShape(SMesh mesh, vector<uint32_t> VIndex);

    // 5. CGAL 点云转 PCL 点云, 并设定颜色
    PCLCloud SubPCLCloudCreatFromShape(const Point_set& Points, Color color);

    // 6. 平均法线计算和，中心点提取
    ShapeNormal ShapeNormalAver(PCLCloud Cloud);

    // 7. 匹配适合抓取的两个面
    vector< vector<int> > MatchPlane(vector<ShapeNormal> SN_Set, int angleD);

    // 点云重建
    SMesh Reconstruction(const Point_set& points, unsigned int iterations,
                  unsigned int neighbors, unsigned int fitting, unsigned int monge, const bool AlphaShape);

    // 抓取点预测
    bool GrsapPosePredict(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud0, 
                                    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud1,
                                    pcl::Normal N, double L);

    // ===============其他函数
    // 矢量化
    pcl::Normal Vectorization(pcl::PointXYZ P);

    // 采样保存
    void SaveNormalAngle(PCLCloud Cloud, string str);

    // 计算包围框
    Bbox ComputerBbox(SMesh mesh);

    // 计算两平行面间的距离
    vector<double> DistanceTwoPlane(pcl::PointXYZ PA, pcl::PointXYZ PB, pcl::Normal NL1);

    // 以及按X 分行，按照y 大小进行排序
    void CloudSortY(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>>& FirstPlane);

    // 给出点云集合，生成边界点云
    pcl::PointCloud<pcl::PointXYZRGBNormal> GenertCloudFromSet(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> PlaneCloud);

    //  两个边界相交区域求解
    pcl::PointCloud<pcl::PointXYZRGBNormal> CrossArea(pcl::PointCloud<pcl::PointXYZRGBNormal> ACloud, pcl::PointCloud<pcl::PointXYZRGBNormal> BCloud);

    // 计算目标点云的最小包围框和目标的中心坐标系
    // 返回目标的位姿和坐标系旋转参数
    // 最小包围框可视化
    Eigen::Matrix3f boundingBoxcloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZRGBNormal> object_cloud);

    // //  边界区域细分 精度 0.001
    pcl::PointCloud<pcl::PointXYZRGBNormal> CloudRefine(pcl::PointCloud<pcl::PointXYZRGBNormal> LCloud);

    // //  查找点云在X轴上近似点的索引值
    vector<size_t> Index_X(pcl::PointCloud<pcl::PointXYZRGBNormal> Cloud, double X, size_t IS);



private:
    // 定义保存，不让用，可以直接调用Smoothing
    void ShapeSmooth(SMesh& mesh, int nb_iterations, double time);
    void MeshSmooth(SMesh& mesh, int nb_iterations);

    

};

CloudMesh::CloudMesh(){
    ;
}

CloudMesh::~CloudMesh(){
    ;
}

// 显示
void CloudMesh::draw(){
    CGAL::draw(MeshInput);
}

// 显示重载，输入SMesh模型
void CloudMesh::draw(SMesh mesh){
    CGAL::draw(mesh);
}

// 显示重载，输入点云集模型
void CloudMesh::draw(Point_set PointSet){
    CGAL::draw(PointSet);
}

// 【打开文件】
// 文件类型为off文件
// 保存到mesh中
bool CloudMesh::OpenFile(string infile, SMesh& mesh)
{
    std::cout << "【START】 Read file to mesh. " <<std::endl;
    string fileInputPath = infile;
    if(!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(fileInputPath, mesh))
    {
        mesh.clear();
        std::cout << "【ERROR】: cannot read the file polygon_mesh.off!" << std::endl;
        std::vector<Point_3> T_Points;
        typedef std::vector<std::vector<std::size_t>> Polygons;
        Polygons T_Polygons;
        Color Vcolor,Fcolor;
        SMesh mymesh;
        std::cout << "Try read as soup Polygons!" << std::endl;
        bool result =CGAL::IO::read_polygon_soup(fileInputPath, T_Points, T_Polygons);
        if (result)
        {
            CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(T_Points, T_Polygons, mymesh);
            typedef boost::property_map<SMesh, CGAL::dynamic_face_property_t<std::size_t> >::type Fccmap;
            Fccmap fccmap = get(CGAL::dynamic_face_property_t<std::size_t>(), mymesh);
            std::cout << PMP::connected_components(mymesh, fccmap) << " CCs before merge." << std::endl;
            PMP::merge_reversible_connected_components(mymesh);
            mesh = mymesh;
            std::cout << "【SUCCESS】: Read data and convert from: " <<  FileInputPath << std::endl;
            return true;
        }
        else
            std::cout << "【ERROR】: read as soup Polygons!" << std::endl;
            return false;
    }
    std::cout << "【SUCCESS】: Read data " << std::endl;
    return true;
}


// 【打开文件】重载
// 文件类型为ply文件
// 保存到Point_set中
bool CloudMesh::OpenFile(string infile, Point_set& MyPointSet)
{
    std::cout << "【START】:  Read file to Point_Set!" << std::endl;
    const string filename1 = infile;
    Point_set m_point_set;

    // 读入点云
    if (!(CGAL::IO::read_PLY(filename1, MyPointSet)))
    {
        std::cerr << "【ERROR】: cannot read file" << std::endl;
        return false;
    }
    std::cout << "【SUCCESS】: Read Points " << std::endl;
    return true;

}

// ====================================================
// 形状平滑  input 迭代次数和时间
void CloudMesh::ShapeSmooth(SMesh& mesh, int nb_iterations, double time)
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
// 形状平滑  input 迭代次数
void CloudMesh::MeshSmooth(SMesh& mesh, int nb_iterations)
{
    typedef CGAL::dynamic_vertex_property_t<bool>                               Vertex_bool_property;
    typedef typename boost::property_map<SMesh, Vertex_bool_property>::type     VCMap;
    typedef boost::graph_traits<SMesh>::vertex_descriptor                       vertex_descriptor;

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

// ========================================================
// 模型平滑---可选择模式（形状平滑还是网格平滑）
// Smoothing(SMesh mesh, int iter, double time)
// sample:  mesh true 3 true 1 0.00001
// ========================================================
SMesh CloudMesh::Smoothing(SMesh mesh, bool MeshS, const int Miter, bool ShapeS, const int Siter, const double time){
    std::cout << "【START】: Smoothing!" << std::endl;
    SMesh pmesh  = mesh;
    if(MeshS && ShapeS)
    {
        MeshSmooth(pmesh, int(ceil(Miter/2)));
        ShapeSmooth(pmesh, Siter, time);
        MeshSmooth(pmesh, int(ceil(Miter/2)));
    }
    else if(MeshS)
    {
        MeshSmooth(pmesh, Miter);
    }
    else
        ShapeSmooth(pmesh, Siter, time);

    return pmesh;
}

// 网格形状检测
// 返回mesh中各个面对应的索引值
vector<vector<uint32_t>> CloudMesh::ShapeDetect(SMesh mesh, double max_dis=1.0, int max_angle=30, int min_region_size = 10){
    std::cout << "【START】:  Shape detect!" << std::endl;

    const Face_range face_range = faces(mesh);
    std::cout <<"* input mesh with "<< face_range.size() << " faces is loaded" << std::endl;
    // Default parameter values for the data file polygon_mesh.off.
    // Eplison 1.91816
    // Normal Tol in degrees 30
    // Minimum Number of Items 322
    const FT          max_distance_to_plane = FT(max_dis);      // 0.01 * 距离
    const FT          max_accepted_angle    = FT(max_angle);
    const std::size_t min_region_size_       = min_region_size;              // 0.01 * 所有面数

    Neighbor_query neighbor_query(mesh);
    const Vertex_to_point_map vertex_to_point_map(get(CGAL::vertex_point, mesh));
    Region_type region_type(mesh,max_distance_to_plane, max_accepted_angle, min_region_size_, vertex_to_point_map);
    // Sort face indices.
    Sorting sorting(mesh, neighbor_query, vertex_to_point_map);
    sorting.sort();
    // Create an instance of the region growing class.
    Region_growing region_growing( face_range, neighbor_query, region_type, sorting.seed_map());
    // Run the algorithm.
    Regions regions;
    region_growing.detect(std::back_inserter(regions));
    // Print the number of found regions.
    std::cout << "* " << regions.size() << " regions have been found" << std::endl;

    // 定义另一个网格，并用颜色显示
    SMesh FaceMesh;
    FaceMesh = mesh;

    // 定义表面颜色
    const Face_range fr = faces(FaceMesh);
    typename SMesh::Property_map<Face_index, Color> face_color;

    // 帮绑定颜色和网格
    bool created;
    boost::tie(face_color, created) = FaceMesh.add_property_map<Face_index, Color>("f:color", Color(0, 0, 0));
    if (!created) {
        std::cout << std::endl <<"region_growing_on_polygon_mesh example finished  Face Color Failed" << std::endl << std::endl;
    }

    // 迭代所有表面
    srand(static_cast<unsigned int>(time(nullptr)));
    int i = 0;
    using size_type = typename SMesh::size_type;
    vector<size_type> ShapePoints;
    vector< vector<size_type> > ShapesIndexs;
    for (const auto& region : regions) {
        ShapePoints.clear();
        // 生成随机颜色
        CGAL::Random rnd(static_cast<unsigned int>(i));
        const Color color(
                static_cast<unsigned char>( rnd.get_int(0, 255)),
                static_cast<unsigned char>(rnd.get_int(0, 255)),
                static_cast<unsigned char>(  rnd.get_int(0, 255)));
        // Iterate through all region items.
        // std::cout<<i++<<std::endl;
        for (const auto index : region) {
            face_color[Face_index(static_cast<size_type>(index))] = color;
            ShapePoints.push_back(index);
        }
        ShapesIndexs.push_back(ShapePoints);
        i++;
    }
    std::cout << "【SUCESS】:  Shape detect!" << std::endl;
    return ShapesIndexs;
}

// 提取网格和其法线
Point_set CloudMesh::PointSetFromVertices(SMesh mesh){
    Point_set m_points;
    m_points.add_normal_map();
    boost::graph_traits<SMesh>::vertex_iterator v;
    int i=0;
    for (v = const_cast<SMesh&>(mesh).vertices_begin();  v != const_cast<SMesh&>(mesh).vertices_end(); v++)
    {
        boost::graph_traits<SMesh>::vertex_descriptor vd(*v);
        const Kernel::Point_3& p = mesh.point(vd);
        Kernel::Vector_3 n = CGAL::Polygon_mesh_processing::compute_vertex_normal(vd, mesh);
        m_points.insert(p,n);
    }
    // std::string output_filename("../../data/normal_out_polygon_mesh.ply");
    // if (!CGAL::IO::write_PLY(output_filename, m_points))
    // {
    //     std::cerr << "Error: cannot write file " << output_filename << std::endl;
    //     return EXIT_FAILURE;
    // }

    return m_points;
}


// 输入网格和面索引提取子网格
// input mesh and Index extra sub mesh
// 【实现】提取出形状ID 生成子点云集
Point_set CloudMesh::SubMeshCreatFromShape(SMesh mesh, vector<uint32_t> VIndex){
    std::cout << "【START】: Extract points from mesh!" << std::endl;
    // SMesh SubMesh;
    const Face_range face_range = faces(mesh);
    std::cout <<"* input mesh with "<< face_range.size() << " faces is loaded" << std::endl;


    Point_set m_points;
    m_points.point_push_map();
    m_points.add_normal_map();
    using size_type = typename SMesh::size_type;
    for (auto i : VIndex)
    {
        auto id = Face_index(static_cast<size_type>(i));
        auto f = mesh.halfedge(id);
        auto VI = mesh.target(f);

        auto point = mesh.point(VI);
        // m_points.insert(point);

        // boost::graph_traits<SMesh>::vertex_descriptor vd(VI);
        // const Kernel::Point_3& p = mesh.point(VI);
        Kernel::Vector_3 n = CGAL::Polygon_mesh_processing::compute_vertex_normal(VI, mesh);
        m_points.insert(point,n);
    }
    std::cout << "【SUCESS】: Extract points from mesh!" << std::endl;
    return m_points;
}

// 输入CGAL 点云
// 转为PCL 带颜色的点云平面待法线
PCLCloud CloudMesh::SubPCLCloudCreatFromShape(const Point_set& Points, Color color){
    std::cout << "【START】: Extract pcl cloud from cgal point_set!" << std::endl;

    PCLCloud cloud;
    // typedef CGAL::Point_set_3<Kernel::Point_3, Kernel::Vector_3>::Index Point_set_index;
    for (Point_set::const_iterator it = Points.begin(); it != Points.end(); ++ it){
        pcl::PointXYZRGBNormal pt; 
        pt.x = Points.point(*it).x();
        pt.y = Points.point(*it).y();
        pt.z = Points.point(*it).z();
        pt.normal_x = Points.normal(*it).x();
        pt.normal_y = Points.normal(*it).y();
        pt.normal_z = Points.normal(*it).z();
        pt.r = color.r();
        pt.g = color.g();
        pt.b = color.b();
        cloud.points.push_back(pt);
    }
    std::cout << "【SCUESS】: Extract pcl cloud!" << std::endl;
    return cloud;
}


// 输入一个带法线的平面
// 返回一个该面的边界和拟合法线
ShapeNormal CloudMesh::ShapeNormalAver(PCLCloud Cloud){
    std::cout << "【START】: Avera Normal computer!" << std::endl;

    pcl::Normal SNormal;
    pcl::PointXYZ A,B,C;

    int PointsSize = Cloud.size();
    int errpointsize = 0;
    srand(static_cast<int>(time(nullptr)));

    // =========== 1.随机中间法线========
    // CGAL::Random rnd(static_cast<int>(41));
    // int index = rnd.get_int(0, PointsSize);
    // A.x = Cloud[index].normal_x;
    // A.y = Cloud[index].normal_y;
    // A.z = Cloud[index].normal_z;
    // int flag = PointsSize<1000 ? PointsSize : int(PointsSize);
    // for (size_t i = 0; i < flag; i++)
    // {
    //     CGAL::Random rnd(static_cast<int>(i));
    //     int index = rnd.get_int(0, PointsSize);
    //     B.x = Cloud[index].normal_x;
    //     B.y = Cloud[index].normal_y;
    //     B.z = Cloud[index].normal_z;
    //     C.x = 0.5 * (A.x + B.x);
    //     C.y = 0.5 * (A.y + B.y);
    //     C.z = 0.5 * (A.z + B.z);
    //     SNormal = Vectorization(C);
    //     A.x = SNormal.normal_x;
    //     A.y = SNormal.normal_y;
    //     A.z = SNormal.normal_z;
    // }

    // =========== 2. 通过所有的坐标差来判断========
    for (size_t i = 0; i < PointsSize; i++)
    {

        SNormal.normal_x += Cloud[i].normal_x;
        SNormal.normal_y += Cloud[i].normal_y;
        SNormal.normal_z += Cloud[i].normal_z;
    }
    SNormal.normal_x = SNormal.normal_x / PointsSize;
    SNormal.normal_y = SNormal.normal_y / PointsSize;
    SNormal.normal_z = SNormal.normal_z / PointsSize;
    // A.x = SNormal.normal_x / PointsSize;
    // A.y = SNormal.normal_y / PointsSize;
    // A.z = SNormal.normal_z / PointsSize;
    // SNormal = Vectorization(A);

    // ====================================================

    cout<<"【Start Normal】: ("<< SNormal.normal_x<<", "<< SNormal.normal_y<<", "<< SNormal.normal_z<<")"<<endl;


    // ========== 1. 夹角差来判断========

    // pcl::Normal ENormal = SNormal;
    
    // A.x = SNormal.normal_x;    A.y = SNormal.normal_y;    A.z = SNormal.normal_z;
    // for (int32_t i =0; i<PointsSize; i++)
    // {
    //     // double err = (SNormal.normal_x * Cloud[i].normal_x + SNormal.normal_y * Cloud[i].normal_y + SNormal.normal_z * Cloud[i].normal_z);
    //     //             // / pow((pow(SNormal.normal_x,2) +pow(SNormal.normal_y,2)  + pow(SNormal.normal_z,2)), 0.5)
    //     //             // / pow((pow(Cloud[i].normal_x,2) +pow(Cloud[i].normal_y,2)  + pow(Cloud[i].normal_z,2)), 0.5);
    //     // double angle = asin(err)*180/M_PI;
    //     // if (angle < -60.0 || 60.0 < angle)
    //     // {
    //     //     errpointsize++;
    //     //     continue;
    //     // }
    //     B.x = Cloud[i].normal_x;
    //     B.y = Cloud[i].normal_y;
    //     B.z = Cloud[i].normal_z;
    //     C.x = 0.5 * (A.x + B.x);
    //     C.y = 0.5 * (A.y + B.y);
    //     C.z = 0.5 * (A.z + B.z);
    //     ENormal = Vectorization(C);
    //     A.x = ENormal.normal_x;
    //     A.y = ENormal.normal_y;
    //     A.z = ENormal.normal_z;
    // }

    // ====================================================

    // ========== 2. 通过所有的横纵坐标来判断========

    pcl::Normal ENormal, Tnormal = SNormal;
    for (size_t i = 0; i < PointsSize; i++)
    {
        double err= fabs(Tnormal.normal_x-Cloud[i].normal_x)+fabs(Tnormal.normal_y-Cloud[i].normal_y)+fabs(Tnormal.normal_z-Cloud[i].normal_z);
        if(err>0.6 )
        {
            errpointsize++;
            // cout<<" " <<i<<" ";
            continue;
        }
        
        ENormal.normal_x += Cloud[i].normal_x;
        ENormal.normal_y += Cloud[i].normal_y;
        ENormal.normal_z += Cloud[i].normal_z;
        // ENormal.normal_x = 0.5 *(ENormal.normal_x + Cloud[i].normal_x);
        // ENormal.normal_y = 0.5 *(ENormal.normal_x + Cloud[i].normal_y);
        // ENormal.normal_z = 0.5 *(ENormal.normal_x + Cloud[i].normal_z);
        // Tnormal = ENormal;
    }
    // cout<< " MAX ERR "<<max<<endl;
    int Nerr = (PointsSize - errpointsize);
    if (Nerr>0)
    {
        A.x = ENormal.normal_x / Nerr;
        A.y = ENormal.normal_y / Nerr;
        A.z = ENormal.normal_z / Nerr;
        Tnormal.normal_x = A.x;
        Tnormal.normal_y = A.y;
        Tnormal.normal_z = A.z;

        ENormal = Vectorization(A);

    }
    else
        ENormal =SNormal;

    // ====================================================


    cout<<"【ERROR Normal size】: "<< errpointsize<<endl;
    cout<<"【ALL Normal size】: "<< PointsSize<<endl;
    cout<<"【Tnormal Normal】: ("<< Tnormal.normal_x<<", "<< Tnormal.normal_y<<", "<< Tnormal.normal_z<<")"<<endl;
    cout<<"【End Normal】: ("<< ENormal.normal_x<<", "<< ENormal.normal_y<<", "<< ENormal.normal_z<<")"<<endl;

    // PCL函数计算质心
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(Cloud, centroid); // 齐次坐标，（c0,c1,c2,1）
    pcl::PointXYZ PointCenter;
    PointCenter.x = centroid[0];
    PointCenter.y = centroid[1];
    PointCenter.z = centroid[2];
    ShapeNormal R(PointCenter, ENormal);
    return R;
}


// 【三维重建】
// 输入点云 输出mesh
// 参数： 点云 迭代次数 区域点云数量 一般为12 ， 后面两个一般为 2
SMesh Reconstruction(const Point_set& points, unsigned int iterations, unsigned int neighbors, unsigned int fitting, unsigned int monge, const bool AlphaShape){
    std::cout << "【START】 Point reconstruction. " <<std::endl;

    typedef CGAL::Scale_space_surface_reconstruction_3<Kernel> ScaleSpace;
    typedef CGAL::Scale_space_reconstruction_3::Advancing_front_mesher<Kernel> ScaleSpaceAFM;
    typedef CGAL::Scale_space_reconstruction_3::Alpha_shape_mesher<Kernel> ScaleSpaceASM;
    typedef CGAL::Scale_space_reconstruction_3::Jet_smoother<Kernel> ScaleSpaceJS;
    typedef CGAL::Scale_space_reconstruction_3::Weighted_PCA_smoother<Kernel> ScaleSpaceWPS;

    // 定义数据格式
    typedef std::vector<Point_3> Points;
    typedef std::vector<std::size_t> Polygon_3;
    typedef std::vector<Polygon_3> Polygons;
    struct Polygon_soup
    {
        Points points;
        Polygons polygons;
    };
    // 定义新建立的数据
    Polygon_soup soup;
    ScaleSpace reconstruct (points.points().begin(), points.points().end());

    double squared_radius = 0.;

    // neighbors = 16
    // fitting = 2
    // monge = 2
    ScaleSpaceJS smoother(neighbors, fitting, monge);
    reconstruct.increase_scale(iterations, smoother);
    // if (!advancing_front_mesher)
    typedef std::pair<Kernel::Point_3, Kernel::Vector_3> PointVectorPair;
    squared_radius = 0.002;
    // squared_radius = CGAL::compute_average_spacing<CGAL::Parallel_if_available_tag> (points, 16); //CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())


    // advancing_front_mesher 和 Alpha shape 只能二选一，默认是Alpha
    // AlphaShape 设计是否强制闭合
    soup.points.reserve(points.size());
    if(AlphaShape)
    {
        // 最后一个参数强制封闭模型
        ScaleSpaceASM mesher(squared_radius, false, true);
        reconstruct.reconstruct_surface (mesher);

        std::ptrdiff_t num = std::distance( mesher.garbage_begin(  ),
                                                    mesher.garbage_end(  ) );
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

            Polygon_3 new_polygon(3);
            new_polygon[0] = map_i2i[(*it)[0]];
            new_polygon[1] = map_i2i[(*it)[1]];
            new_polygon[2] = map_i2i[(*it)[2]];
            soup.polygons.push_back(new_polygon);
        }
    }
    else
    {
        ScaleSpaceAFM mesher(0.0, 5.0, 30.0);
        reconstruct.reconstruct_surface (mesher);
        soup.polygons.reserve(reconstruct.number_of_facets ());

        std::map<std::size_t, std::size_t> map_i2i;
        std::size_t current_index = 0;

        for (ScaleSpace::Facet_iterator it = reconstruct.facets_begin();
            it != reconstruct.facets_end(); ++ it)
        {
            for (unsigned int ind = 0; ind < 3; ++ ind)
            {
                if (map_i2i.find ((*it)[ind]) == map_i2i.end ())
                {
                    map_i2i.insert (std::make_pair ((*it)[ind], current_index ++));
                    Point_3 p = points.point(*(points.begin() + (*it)[ind]));       // 这句有问题
                    soup.points.push_back(Point_3(p.x (), p.y (), p.z ()));
                }
            }
            Polygon_3 new_polygon(3);
            new_polygon[0] = map_i2i[(*it)[0]];
            new_polygon[1] = map_i2i[(*it)[1]];
            new_polygon[2] = map_i2i[(*it)[2]];
            soup.polygons.push_back(new_polygon);
        }
    }
    SMesh myMesh;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh< CGAL::Surface_mesh<Point_3> >(
                                        soup.points, soup.polygons, myMesh);
    std::size_t rv = CGAL::Polygon_mesh_processing::remove_isolated_vertices(myMesh);
    if(rv > 0){
    std::cerr << "Ignore isolated vertices: " << rv << std::endl;
    myMesh.collect_garbage();
    }
    if(myMesh.vertices().size() > 0)
    std::cout << "【SUCESS】 Point reconstruction. " <<std::endl;
    return myMesh;
}

// 矢量化
pcl::Normal CloudMesh::Vectorization(pcl::PointXYZ P){
    pcl::Normal out;
    double L = pow((pow(P.x,2) + pow(P.y,2) + pow(P.z,2)), 0.5);
    double xyL = pow((pow(P.x,2)+pow(P.y,2)), 0.5);
    double theta = asin(P.y / xyL );
    double alfa = asin(P.x / xyL );
    double bata = asin(P.z / L);
    out.normal_x = cos(bata) * sin(alfa);
    out.normal_y = cos(bata) * sin(theta);
    out.normal_z = sin(bata);
    return out;
}

// 保存点云法向量
void CloudMesh::SaveNormalAngle(PCLCloud Cloud, string str){
    pcl::PointXYZ P;
    fstream out(str, ios::out );
    for (size_t i = 0; i < Cloud.size(); i++)
    {
        // int theta = int(180 /M_PI * atan(Cloud[i].normal_y / Cloud[i].normal_x ));
        // int alfa =  int(180 /M_PI * atan(Cloud[i].normal_x  / Cloud[i].normal_y ));
        // int bata =  int(180 /M_PI * atan(Cloud[i].normal_z));

        double theta = Cloud[i].normal_x;
        double alfa =  Cloud[i].normal_y;
        double bata =  Cloud[i].normal_z;
        string data = to_string(theta) + " " + to_string(alfa) + " " + to_string(bata) + "\n" ;
        out<<data;
    }
    out.close();
}


// 计算mesh的最小包围框
Bbox CloudMesh::ComputerBbox(SMesh mesh){
    std::cout << "【START】 Computer Bbox. " <<std::endl;
    // Bbox x_min, y_min, z_min, x_max, y_max, z_max
    double x_min, y_min, z_min, x_max, y_max, z_max;
    boost::graph_traits<SMesh>::vertex_iterator v = const_cast<SMesh&>(mesh).vertices_begin();
    boost::graph_traits<SMesh>::vertex_descriptor vd(*v);
    Kernel::Point_3& p = mesh.point(vd);
    x_min = p.x();
    y_min = p.y();
    z_min = p.z();
    x_max = y_max = z_max =0;
    for (v++;  v != const_cast<SMesh&>(mesh).vertices_end(); v++)
    {
        vd = *v ;
        p = mesh.point(vd);
        x_min = x_min < p.x() ? x_min : p.x();
        y_min = y_min < p.y() ? y_min : p.y();
        z_min = z_min < p.z() ? z_min : p.z();
        x_max = x_max > p.x() ? x_max : p.x();
        y_max = y_max > p.y() ? y_max : p.y();
        z_max = z_max > p.z() ? z_max : p.z();
    }
    Bbox box(x_min, y_min, z_min, x_max, y_max, z_max);
    return box;
}



// 输入平面的法线集
// 返回匹配到的平面集合
// 返回形式为一个带有两个下标索引对应的集合
vector< vector<int> > CloudMesh::MatchPlane(vector<ShapeNormal> SN_Set, int angleD=10){
    
    // 寻找相同法线的面
    vector< vector<int> > EqList;
    vector<int> Index;
    for (size_t i = 0; i < SN_Set.size(); i++)
        Index.push_back(i);
    
    for(int i=0; i<Index.size(); i++){
        vector<int> shape;
        pcl::Normal N1;
        if(Index[i] == -1)
            continue;
        shape.push_back(Index[i]);
        N1 = SN_Set[Index[i]].second;
        for (size_t j = i+1; j < Index.size(); j++)
        {
            if(Index[j] == -1)
                continue;
            pcl::Normal N2;
            N2 = SN_Set[Index[j]].second;

            // 计算法线角度
            double err = N1.normal_x * N2.normal_x + N1.normal_y * N2.normal_y + N1.normal_z * N2.normal_z;
            double angleflag = acos(-1) * angleD / 180 ;
            if (err > cos(angleflag)) // 10度范围内
            {
                shape.push_back(Index[j]);
                Index[j] = -1;
            }
        }
        EqList.push_back(shape);
    }
    cout<<"EqList Size: " << EqList.size() <<endl;

    // 输出找到相同的面索引值
    Index.clear();
    for (size_t i = 0; i < EqList.size(); i++)
        Index.push_back(i);

    vector< vector<int> > InvqList_temp;
    for (size_t i = 0; i < Index.size(); i++)
    {
        pcl::Normal N1;
        if(Index[i] == -1)
            continue;
            
        N1 = SN_Set[EqList[Index[i]][0]].second;
        for (size_t j = 0; j < Index.size(); j++)
        {
            if(Index[j] == -1)
                continue;
            pcl::Normal N2;
            N2 = SN_Set[EqList[Index[j]][0]].second;

            // 计算法线角度
            double err = N1.normal_x * N2.normal_x + N1.normal_y * N2.normal_y + N1.normal_z * N2.normal_z;
            if (err < -0.984807753) // 10度范围内
            {
                vector<int> temp = {Index[i], Index[j]};
                InvqList_temp.push_back(temp);
                Index[j] = -1;
            }
        }
    }
    cout<<"InvqList_temp Size: " << InvqList_temp.size() <<endl;

    // 保存真正对立面的索引值
    vector< vector<int> > InvqList;
    for (size_t i = 0; i < InvqList_temp.size(); i++)
    {
        if(EqList[ InvqList_temp[i][0] ].size()>1 | EqList[ InvqList_temp[i][1] ].size()>1)
        {
            // 对立面索引中只存在多个面
            // 寻找距离最小的面
            vector<int> V1 = EqList[InvqList_temp[i][0] ];
            vector<int> V2 = EqList[InvqList_temp[i][1] ];
            
            double MaxDistance = pow( (
                    pow((SN_Set[V1[0]].first.x - SN_Set[V2[0]].first.x), 2) + 
                    pow((SN_Set[V1[0]].first.y - SN_Set[V2[0]].first.y), 2) + 
                    pow((SN_Set[V1[0]].first.z - SN_Set[V2[0]].first.z), 2) ) ,0.5);
            int Index1 = V1[0], Index2 = V2[0];
            for (size_t i = 0; i < V1.size(); i++)
            {
                pcl::PointXYZ P1 = SN_Set[V1[i]].first;
                for (size_t j = 0; j < V2.size(); j++)
                {
                    pcl::PointXYZ P2 = SN_Set[V2[j]].first;
                    double Distance = pow( (
                        pow((P1.x - P2.x), 2) + 
                        pow((P1.y - P2.y), 2) + 
                        pow((P1.z - P2.z), 2) ) ,0.5);
                    if (Distance < MaxDistance)
                    {
                        MaxDistance = Distance;
                        Index1 = V1[i];
                        Index2 = V2[j];
                    }
                }
            }
            vector<int> t = {Index1, Index2};
            InvqList.push_back(t); 
        }
        else
        {
            InvqList.push_back(InvqList_temp[i]); // 对立面索引中只存在两个面
        }
    }

    // 返回保存对立面最小距离的集合
    return InvqList;
}

// 输入两个平面点云指针
// 返回两个平面之间的距离
vector<double> CloudMesh::DistanceTwoPlane(pcl::PointXYZ PA, pcl::PointXYZ PB, pcl::Normal NL1){
    // 定义点C 过点A 在L2 平面的垂线交点为C
    // H 是AC 间的距离，也就是最后求的未知数
    // 方程如下，手动接得H = 1 或 0，取1
    // x = A.x + H * a
    // y = A.y + H * b
    // z = A.z + H * c
    // |AB|^2 = |AC|^2 + |BC|^2
    pcl::PointXYZ PC;
    double x1,x2,x3,y1,y2,y3,z1,z2,z3;
    double a,b,c;
    double H;
    x1 = PA.x; y1 = PA.y; z1 = PA.z;
    x2 = PB.x; y2 = PB.y; z2 = PB.z;
    a = NL1.normal_x; b = NL1.normal_y; c = NL1.normal_z;
    double X, Y, Z;
    X = x1-x2; Y = y1-y2; Z = z1-z2;
    
    double E = 2 * (pow(a,2) + pow(b,2) + pow(c,2));
    double F = -2 * (X*a + Y*b + Z*c);
    double alfa2 = pow((x1-x2), 2) + pow((y1-y2), 2) + pow((z1-z2), 2);
    double G = pow(X,2) + pow(Y,2) + pow(Z,2) - alfa2;

    double Htemp = pow((pow(F,2) - 4*E*G), 0.5);
    H = 0.5 * (Htemp - F) / E;
    x3 = x1 - H * a;
    y3 = y1 - H * b;
    z3 = z1 - H * c;
    PC.x = x3;
    PC.y = y3;
    PC.z = z3;
    vector<double> r;
    r.push_back(H);
    r.push_back(x3);
    r.push_back(y3);
    r.push_back(z3);
    return r;
}


// 输入两个平面点云指针、一个法线、两个面之间的距离
// 预测抓取姿态
bool CloudMesh::GrsapPosePredict(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud0, 
                                    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud1,
                                    pcl::Normal N, double L){
    // 两个平面按 x y 排序
    // 寻找两个面的边界点
    // 相交区域求解
    // 抓取姿态预测

    // 【1】点云排序=========
    // pcl::PointCloud<pcl::PointXYZRGBNormal> cloud0, cloud1;
    // pcl::copyPointCloud(*Ptr0, cloud0);
    // pcl::copyPointCloud(*Ptr1, cloud1);
    std::cout<<std::endl<<"【START】 Grasp Predtic! "<<std::endl;


    pcl::PointXYZRGBNormal Lmin;//用于存放三个轴的最小值
    pcl::PointXYZRGBNormal Lmax;//用于存放三个轴的最大值
    pcl::getMinMax3D(cloud0, Lmin, Lmax);

    pcl::PointXYZRGBNormal Rmin;//用于存放三个轴的最小值
    pcl::PointXYZRGBNormal Rmax;//用于存放三个轴的最大值
    pcl::getMinMax3D(cloud1, Rmin, Rmax);

    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> FirstPlane;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> SecondPlane;
    int FirstPlaneCount, SecondPlaneCount;
    FirstPlaneCount = int((Lmax.x - Lmin.x) * 100) + 1; // 实际是 / 0.01 向上取整
    SecondPlaneCount = int((Rmax.x - Rmin.x) * 100) + 1;
    FirstPlane.resize(FirstPlaneCount);
    SecondPlane.resize(SecondPlaneCount);
    for(size_t i=0; i<cloud0.size(); i++){
        int x_index;
        x_index = int(100 * (cloud0[i].x - Lmin.x)); // 实际是 / 0.01  向下取整
        FirstPlane[x_index].push_back(cloud0[i]);
    }
    for(size_t i=0; i<cloud1.size(); i++){
        int x_index;
        x_index = int(100 * (cloud1[i].x - Rmin.x)); // 实际是 / 0.01  向下取整
        SecondPlane[x_index].push_back(cloud1[i]);
    }

    // 对两个面按照y从升序排列
    CloudSortY(FirstPlane);
    CloudSortY(SecondPlane);

    pcl::PointCloud<pcl::PointXYZRGBNormal> BoardCloudL = GenertCloudFromSet(FirstPlane);
    pcl::PointCloud<pcl::PointXYZRGBNormal> BoardCloudR = GenertCloudFromSet(SecondPlane);
    // 画边界线(原始)
    static int Linei=0;
    for(int j=1; j<BoardCloudL.size(); j++)
            viewer->addLine(BoardCloudL[j-1], BoardCloudL[j], 1.0, 1.0, 1.0, to_string(Linei) + "L_border_Line_" + to_string(j));
    viewer->addLine(BoardCloudL[BoardCloudL.size()-1], BoardCloudL[0], 1.0, 1.0, 1.0, to_string(Linei) + "L_border_Line_" + to_string(0));

    for(int j=1; j<BoardCloudR.size(); j++)
            viewer->addLine(BoardCloudR[j-1], BoardCloudR[j], 1.0, 1.0, 1.0,  to_string(Linei) + "R_border_Line_" + to_string(j));
    viewer->addLine(BoardCloudR[BoardCloudR.size()-1], BoardCloudR[0], 1.0, 1.0, 1.0,  to_string(Linei) + "R_border_Line_" + to_string(0));
    

    // ==========================
    // 选出最小面，在另外一面生成投影面

     pcl::PointCloud<pcl::PointXYZRGBNormal> ACloud, BCloud;
    if(BoardCloudL.size()<= BoardCloudR.size())
    {
        BCloud = BoardCloudL;
        ACloud = BoardCloudR;
        
    }
    else
    {
        BCloud = BoardCloudR;
        ACloud = BoardCloudL;
        N.normal_x = -N.normal_x;
        N.normal_y = -N.normal_y;
        N.normal_z = -N.normal_z;

    }
    pcl::PointCloud<pcl::PointXYZRGBNormal> ProjectionCloud;
    for (size_t i = 0; i < BCloud.size(); i++)
    {
        pcl::PointXYZRGBNormal P = BCloud[i];
        P.x = BCloud[i].x - N.normal_x * L;
        P.y = BCloud[i].y - N.normal_y * L;
        P.z = BCloud[i].z - N.normal_z * L;
        ProjectionCloud.push_back(P);
    }
    ProjectionCloud.width = ProjectionCloud.size();
    ProjectionCloud.height = 1;

    // 画边界线(投影)
    for(int j=1; j<ProjectionCloud.size(); j++)
            viewer->addLine(ProjectionCloud[j-1], ProjectionCloud[j], 0.0, 0.0, 1.0, to_string(Linei) + "ProjectionCloud_Line_" + to_string(j));
    viewer->addLine(ProjectionCloud[ProjectionCloud.size()-1], ProjectionCloud[0], 0.0, 0.0, 1.0, to_string(Linei) + "ProjectionCloud_Line_" + to_string(0));
    Linei++;
    pcl::PointCloud<pcl::PointXYZRGBNormal> CloudRefineProje = CloudRefine(ProjectionCloud);
    pcl::PointCloud<pcl::PointXYZRGBNormal> CloudRefineBase = CloudRefine(ACloud);


    // 相交的边界点，里面所有的点都可以直接用作取抓取的点
    pcl::PointCloud<pcl::PointXYZRGBNormal> CrossCloud = CrossArea(CloudRefineBase,  CloudRefineProje);
    if(CrossCloud.size()>0)
    {
        // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CrossCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        // pcl::copyPointCloud(CrossCloud, *CrossCloudPtr);
        // viewer->addPointCloud<pcl::PointXYZRGBNormal>(CrossCloudPtr, "CrossCloudPtr");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "CrossCloudPtr");//设置点云大
        for(int j=1; j<CrossCloud.size(); j++)
                viewer->addLine(CrossCloud[j-1], CrossCloud[j], 1.0, 1.0, 0.0, "CrossCloud_Line_"+ to_string(j));
        viewer->addLine(CrossCloud[CrossCloud.size()-1], CrossCloud[0], 1.0, 1.0, 0.0, "CrossCloud_Line_"+ to_string(0));

    }
    else
        return -1;

    // 抓取点采样
    // 记住相交线点云的存储方式，从中间砍半并对称。
    // 取对应两个点的中间点做为一个面的抓取点，求另一个面对应的抓取点做
    int grasp_index = 0;
    pcl::PointXYZRGBNormal PA = CrossCloud[grasp_index];
    pcl::PointXYZRGBNormal PB = PA;
    PB.x = PA.x + N.normal_x * L;
    PB.y = PA.y + N.normal_y * L;
    PB.z = PA.z + N.normal_z * L;   

    // 确定向量  X 方向
    Eigen::Matrix3f eigenVectorsPCA = boundingBoxcloud(viewer, BCloud);
	pcl::Normal pcZ;
	pcZ.normal_x = eigenVectorsPCA(0, 1);
	pcZ.normal_y = eigenVectorsPCA(1, 1);
	pcZ.normal_z = eigenVectorsPCA(2, 1);

    // 通过 X 和 N 标准化并确定第三轴
    Eigen::Matrix3d DisM = Eigen::Matrix3d::Identity();
    DisM(0, 0) = pcZ.normal_x;
    DisM(1, 0) = pcZ.normal_y;
    DisM(2, 0) = pcZ.normal_z;
    std::cout << "旋转向量ve(3x3):\n" << DisM << std::endl;
    DisM(0, 1) = N.normal_x;
    DisM(1, 1) = N.normal_y;
    DisM(2, 1) = N.normal_z;
    std::cout << "旋转向量ve(3x3):\n" << DisM << std::endl;
    DisM.col(0).normalize();
    std::cout << "旋转向量ve(3x3):\n" << DisM << std::endl;
    DisM.col(1).normalize();
    std::cout << "旋转向量ve(3x3):\n" << DisM << std::endl;
    DisM.col(2) = DisM.col(0).cross(DisM.col(1)); //校正主方向间垂直  叉乘 求法线
	DisM.col(1) = DisM.col(2).cross(DisM.col(0));
	DisM.col(0) = DisM.col(1).cross(DisM.col(2));


	std::cout << "旋转向量ve(3x3):\n" << DisM << std::endl;

    // 法线方向+旋转角度作为姿态
    Eigen::Vector3d position_ ;
    position_[0] = (PA.x + PB.x) / 2;
    position_[1] = (PA.y + PB.y) / 2;
    position_[2] = (PA.z + PB.z) / 2;


    double outer_diameter   = 0.075 + L;          // 控制抓取宽度
    double finger_width     = 0.025;            // 指的宽度
    double hand_depth       = 0.06;             // 指的长度
    double hand_height      = 0.02;             // 指的厚度
    int idx                 = 0;
    const Eigen::Vector3d rgb(1.0, 0, 0);
    ToolPlot::plotHand3D(viewer, position_, DisM, outer_diameter, finger_width, hand_depth, hand_height,idx, rgb);

    // 抓取点显示
    pcl::PointXYZ graspA,graspB;
    graspA.x = PA.x; graspA.y = PA.y; graspA.z = PA.z;
    graspB.x = PB.x; graspB.y = PB.y; graspB.z = PB.z;
    viewer->addSphere(graspA, 0.0025, "GraspPoseA");
    viewer->addSphere(graspB, 0.0025, "GraspPoseB");
    pcl::PointXYZ ArrowA,ArrowB;
    ArrowA.x = position_[0]; ArrowA.y = position_[1]; ArrowA.z = position_[2]; 
    ArrowB.x = ArrowA.x + 0.1*N.normal_x;
    ArrowB.y = ArrowA.y + 0.1*N.normal_y;
    ArrowB.z = ArrowA.z + 0.1*N.normal_z;
    viewer->addArrow<pcl::PointXYZ>(ArrowB, ArrowA, 255, 0, 0, false, "End_arrow");


    std::cout<<"【SUCCESS】 Grasp Predtic! "<<std::endl;
    return true;
}





// 点云排序
// 以及按X 分行，按照y 大小进行排序
void CloudMesh::CloudSortY(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>>& FirstPlane)
{
    for (size_t i = 0; i < FirstPlane.size(); i++)
    {
        // pcl::PointXYZ imin, imax;//用于存放三个轴的最小值
        // pcl::getMinMax3D(FirstPlane[i], imin, imax);
        
        //升序排序
        std::sort(
            FirstPlane[i].begin(),
            FirstPlane[i].end(),
            [](const pcl::PointXYZRGBNormal & lhs, const pcl::PointXYZRGBNormal& rhs)->bool //如果第一个参数 < 第二个参数，则返回真
        {
            //需要满足严格弱序的条件，何谓严格弱序？根据标准库手册的描述，comp需要满足如下属性：
            //  - 对于任一变量a，comp(a, a)为false
            //  - 如果comp(a, b) 为true，那么comp(b, a)为false
            //  - 如果comp(a, b)为true且comp(b, c)true，则comp(a, c)为true

            //排序方式，优先级：x轴优先级  > y轴 > z轴
            if (lhs.y < rhs.y) return true;
            else if (lhs.y > rhs.y) return false;
            else if (lhs.x < rhs.x) return true;
            else if (lhs.x > rhs.x) return false;
            else if (lhs.z < rhs.z) return true;
            else return false;
        });
    }
}

// 给出点云集合，生成边界点云
pcl::PointCloud<pcl::PointXYZRGBNormal> CloudMesh::GenertCloudFromSet(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> PlaneCloud)
{
    // 查找第一个面的边界线
    pcl::PointCloud<pcl::PointXYZRGBNormal> BoardCloudL;
    // 添加第一行
    BoardCloudL = PlaneCloud[0];
    // 添加右侧列
    for (size_t i = 1; i < PlaneCloud.size()-1; i++)
    {
        pcl::PointXYZRGBNormal P2;
        int index = PlaneCloud[i].size()-1;
        P2.x = PlaneCloud[i][index].x; P2.y = PlaneCloud[i][index].y; P2.z = PlaneCloud[i][index].z;
        P2.r = 1.0;
        P2.g = 0.0;
        P2.b = 0.0;
        P2.normal_x = 1.0;
        P2.normal_y = 0.0;
        P2.normal_z = 0.0;
        BoardCloudL.push_back(P2);
    }
    // 添加第尾行 ，倒着插入
    int sizeLast = PlaneCloud.size()-1;
    for (size_t i = PlaneCloud[sizeLast].size()-1; i > 0; i--)
    {
        pcl::PointXYZRGBNormal P1;
        P1.x = PlaneCloud[sizeLast][i].x; P1.y = PlaneCloud[sizeLast][i].y; P1.z = PlaneCloud[sizeLast][i].z;
        P1.r = 1.0;
        P1.g = 0.0;
        P1.b = 0.0;
        P1.normal_x = 1.0;
        P1.normal_y = 0.0;
        P1.normal_z = 0.0;
        BoardCloudL.push_back(P1);
    }
    // 添加左侧侧列，倒着插入
    for (size_t i = PlaneCloud.size()-2; i > 0; i--)
    {
        pcl::PointXYZRGBNormal P1;
        P1.x = PlaneCloud[i][0].x; P1.y = PlaneCloud[i][0].y; P1.z = PlaneCloud[i][0].z;
        P1.r = 1.0;
        P1.g = 0.0;
        P1.b = 0.0;
        P1.normal_x = 1.0;
        P1.normal_y = 0.0;
        P1.normal_z = 0.0;
        BoardCloudL.push_back(P1);
    }
    BoardCloudL.width = BoardCloudL.size();
    BoardCloudL.height = 1;
    std::cout<<"board size "<<PlaneCloud.size() <<std::endl;
    return BoardCloudL;
}

//  边界区域细分 精度 0.001
pcl::PointCloud<pcl::PointXYZRGBNormal> CloudMesh::CloudRefine(pcl::PointCloud<pcl::PointXYZRGBNormal> LCloud)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal> LCloudNew;
    for (size_t i = 1; i < LCloud.size(); i++)
    {
        pcl::PointXYZRGBNormal P1 = LCloud[i-1];
        pcl::PointXYZRGBNormal P2 = LCloud[i];
        pcl::PointXYZRGBNormal P = LCloud[i-1];
        double L = pow( (pow((P1.x-P2.x), 2) + pow((P1.y-P2.y), 2)  + pow((P1.z-P2.z), 2)) , 0.5);
        int count_s = L / 0.001; 
        count_s = (count_s <= 1 ? 1 : count_s);
        double Lx = (P2.x - P1.x) / count_s;
        double Ly = (P2.y - P1.y) / count_s;
        double Lz = (P2.z - P1.z) / count_s;

        LCloudNew.push_back(P1);
        P.r = 1.0; P.g = 1.0; P.b = 1.0;
        for(size_t j = 1; j<count_s; j++)
        {
            P.x = P1.x + Lx * j;
            P.y = P1.y + Ly * j;
            P.z = P1.z + Lz * j;
            LCloudNew.push_back(P);
        }
    }
    LCloudNew.width = LCloudNew.size();
    LCloudNew.height =1;
    return LCloudNew;
}


//  查找点云在X轴上近似点的索引值
vector<size_t> CloudMesh::Index_X(pcl::PointCloud<pcl::PointXYZRGBNormal> Cloud, double X, size_t IS = 0)
{
    vector<size_t> R;
    for (size_t i = IS; i < Cloud.size(); i++)
    {
        double err = abs(Cloud[i].x - X);
        if (err<= 0.001)
            R.push_back(i);
        else if(R.size()>=1)
            return R;
    }
    R.push_back(-1);
    return R;
}


//  两个边界相交区域求解
// 输入 A B 两个点云
pcl::PointCloud<pcl::PointXYZRGBNormal> CloudMesh::CrossArea(pcl::PointCloud<pcl::PointXYZRGBNormal> ACloud, pcl::PointCloud<pcl::PointXYZRGBNormal> BCloud)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal> LCloudNew;
    pcl::PointXYZRGBNormal Amin;//用于存放三个轴的最小值
    pcl::PointXYZRGBNormal Amax;//用于存放三个轴的最大值
    pcl::getMinMax3D(ACloud, Amin, Amax);

    pcl::PointXYZRGBNormal Bmin;//用于存放三个轴的最小值
    pcl::PointXYZRGBNormal Bmax;//用于存放三个轴的最大值
    pcl::getMinMax3D(BCloud, Bmin, Bmax);

    enum xType{XIntersect, XInclud, XSeparated} XType;
    enum yType{YIntersect, YInclud, YSeparated} YType;
    pcl::PointCloud<pcl::PointXYZRGBNormal> LC,RC;

    pcl::PointXYZRGBNormal Lmin;//用于存放三个轴的最小值
    pcl::PointXYZRGBNormal Lmax;//用于存放三个轴的最大值
    pcl::PointXYZRGBNormal Rmin;//用于存放三个轴的最小值
    pcl::PointXYZRGBNormal Rmax;//用于存放三个轴的最大值
    if (Amin.x <= Bmin.x)
    {
        LC = ACloud;
        RC = BCloud;
        Lmin = Amin;
        Lmax = Amax;
        Rmin = Bmin;
        Rmax = Bmax;
    }
    else{
        LC = BCloud;
        RC = ACloud;
        Lmin = Bmin;
        Lmax = Bmax;
        Rmin = Amin;
        Rmax = Amax;
    }

    // 宏观确认两者的关系
    if (Lmax.x <= Rmin.x)
        XType = XSeparated;
    else if(Lmax.x < Rmax.x)
        XType = XIntersect;
    else
        XType = XInclud;

    if (Lmin.y <= Rmin.y)
    {
        if (Lmax.y <= Rmin.y)
            YType = YSeparated;
        else if(Lmax.y < Rmax.y)
            YType = YIntersect;
        else
            YType = YInclud;
    }
    else{
        if (Rmax.y <= Lmin.y)
            YType = YSeparated;
        else if(Rmax.y < Lmax.y)
            YType = YIntersect;
        else
            YType = YInclud;
    }
    if (XType == XSeparated || YType == YSeparated)
        return LCloudNew;

    // X 轴线排序
    std::sort(LC.begin(), LC.end(), [](const pcl::PointXYZRGBNormal & lhs, const pcl::PointXYZRGBNormal& rhs)->bool
    {
            if (lhs.x < rhs.x) return true;
            else if (lhs.x > rhs.x) return false;
            else if (lhs.y < rhs.y) return true;
            else if (lhs.y > rhs.y) return false;
            else if (lhs.z < rhs.z) return true;
            else return false;
    });
    std::sort(RC.begin(), RC.end(), [](const pcl::PointXYZRGBNormal & lhs, const pcl::PointXYZRGBNormal& rhs)->bool
    {
            if (lhs.x < rhs.x) return true;
            else if (lhs.x > rhs.x) return false;
            else if (lhs.y < rhs.y) return true;
            else if (lhs.y > rhs.y) return false;
            else if (lhs.z < rhs.z) return true;
            else return false;
    });
    // string path0 = "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/normal/L.ply";
    // string path1 = "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/normal/R.ply";
    // pcl::io::savePCDFileASCII(path0, LC);
    // pcl::io::savePCDFileASCII(path1, RC);
    
    LCloudNew.clear();
    pcl::PointXYZRGBNormal P;//用于存放三个轴的最小值
    P.r = 1.0;
    P.g = 1.0;
    P.b = 1.0;
    P.normal_x =1.0;
    P.normal_y =0.0;
    P.normal_z =0.0;
    double S_x0,S_x1;
    S_x0 = S_x1 = Rmin.x;
    size_t S_index_0,S_index_1;
    S_index_0 = S_index_1 =0;
    while (true)
    {
        vector<size_t> Lstart = Index_X(LC, S_x0, S_index_0);
        vector<size_t> Rstart = Index_X(RC, S_x1, S_index_1);

        if (S_x0 >= Lmax.x || S_x1 >= Rmax.x || Rstart[Rstart.size()-1] == RC.size())
            break;

        S_x0 = LC[Lstart[Lstart.size()-1]+1].x;
        S_x1 = RC[Rstart[Rstart.size()-1]+1].x;
        S_index_0 = Lstart[Lstart.size()-1] + 1;
        S_index_1 = Rstart[Rstart.size()-1] + 1;
        if (Lstart.size()<2 || Rstart.size()<2|| Lstart[0]<0 || Rstart[0]<0)
            continue;
        
        vector<double> L,R;
        double Lminy, Lmaxy;
        double Rminy, Rmaxy;

        size_t LminI,LmaxI,RminI,RmaxI;

        Lminy = Lmaxy = LC[Lstart[0]].y;
        LminI = LmaxI = Lstart[0];
        for (size_t i = 1; i < Lstart.size(); i++)
        {
            LminI = Lminy <= LC[Lstart[i]].y ? LminI : Lstart[i];
            LmaxI = Lmaxy > LC[Lstart[i]].y ? LmaxI : Lstart[i];
        }

        Rminy = Rmaxy = LC[Lstart[0]].y;
        RminI = RmaxI = Rstart[0];
        for (size_t i = 1; i < Rstart.size(); i++)
        {
            RminI = Rminy <= RC[Rstart[i]].y ? RminI : Rstart[i];
            RmaxI = Rmaxy > RC[Rstart[i]].y ? RmaxI : Rstart[i];
        }
        if (LC[LminI].y > RC[RmaxI].y || LC[LmaxI].y < RC[RminI].y)
            continue;
        else if(LC[LmaxI].y > RC[RmaxI].y)
        {
            P = LC[LminI];
            P.r = 1.0; P.g = 0.0; P.b = 0.0;
            LCloudNew.push_back(P);
            P.x = RC[RmaxI].x;
            P.y = RC[RmaxI].y;
            P.z = RC[RmaxI].z;
            LCloudNew.push_back(P);
            // cout<<" LCloudNew: "<<LCloudNew.size()<<endl;
        }
        else
        {
            P = LC[LmaxI];
            P.r = 1.0; P.g = 0.0; P.b = 0.0;
            LCloudNew.push_back(P);
            P.x = RC[RminI].x;
            P.y = RC[RminI].y;
            P.z = RC[RminI].z;
            LCloudNew.push_back(P);
            // cout<<" LCloudNew: "<<LCloudNew.size()<<endl;
        }
    }
    LCloudNew.width = LCloudNew.size();
    LCloudNew.height = 1;
    pcl::PointCloud<pcl::PointXYZRGBNormal> Cloudtemp;
    for (size_t i = 0; i < (LCloudNew.size()/2 -1); i++)
    {
        Cloudtemp.push_back(LCloudNew[2*i]);
    }
    for (size_t i = (LCloudNew.size()/2 -1); i >0 ; i--)
    {
        Cloudtemp.push_back(LCloudNew[2*i + 1]);
    }
    LCloudNew = Cloudtemp;
    return LCloudNew;
}



// 计算目标点云的最小包围框和目标的中心坐标系
// 返回目标的位姿和坐标系旋转参数
// 最小包围框可视化
Eigen::Matrix3f CloudMesh::boundingBoxcloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZRGBNormal> object_cloud)
{
    Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(object_cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(object_cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

    // std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	// std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	// std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;

    Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();
 
	// std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
	// std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;
 
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::transformPointCloud(object_cloud, *transformedCloud, tm);
 
	pcl::PointXYZRGBNormal min_p, max_p;
	Eigen::Vector3f c1, c;

	pcl::getMinMax3D(object_cloud, min_p, max_p);
	c1 = 0.5f*(min_p.getVector3fMap() + max_p.getVector3fMap());

	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);
 
	Eigen::Vector3f whd;
	whd = max_p.getVector3fMap() - min_p.getVector3fMap();
	float sc1 = (whd(0) + whd(1) + whd(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小
 
 
	//初始点云的主方向
    // geometry_msgs::Point cp;
	pcl::PointXYZ cp;
	cp.x = pcaCentroid(0);
	cp.y = pcaCentroid(1);
	cp.z = pcaCentroid(2);
    // geometry_msgs::Point pcX;
	pcl::PointXYZ pcX;
	pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
	pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
	pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
    // geometry_msgs::Point pcY;
	pcl::PointXYZ pcY;
	pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
	pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
	pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
    // geometry_msgs::Point pcZ;
	pcl::PointXYZ pcZ;
	pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
	pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
	pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;

    static int i=0;
	viewer->addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x_"+std::to_string(i));
	viewer->addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y_"+std::to_string(i));
	viewer->addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z_"+std::to_string(i));
    i++;
    return eigenVectorsPCA;
}

