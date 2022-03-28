// STL includes.
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <iostream>
#include <iterator>
#include <string>
#include <cstdlib>
#include <cmath>
#include <math.h>

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

#include "plot.h"

using namespace std;


// 点云排序
// 以及按X 分行，按照y 大小进行排序
void CloudSortY(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>>& FirstPlane)
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
pcl::PointCloud<pcl::PointXYZRGBNormal> GenertCloudFromSet(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> PlaneCloud)
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
pcl::PointCloud<pcl::PointXYZRGBNormal> CloudRefine(pcl::PointCloud<pcl::PointXYZRGBNormal> LCloud)
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
vector<size_t> Index_X(pcl::PointCloud<pcl::PointXYZRGBNormal> Cloud, double X, size_t IS = 0)
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
pcl::PointCloud<pcl::PointXYZRGBNormal> CrossArea(pcl::PointCloud<pcl::PointXYZRGBNormal> ACloud, pcl::PointCloud<pcl::PointXYZRGBNormal> BCloud)
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
Eigen::Matrix3f boundingBoxcloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZRGBNormal> object_cloud)
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

    std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;

    Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();
 
	std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
	std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;
 
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

	viewer->addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
	viewer->addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
	viewer->addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");
    return eigenVectorsPCA;
}


// 输入两个平面点云指针、一个法线、两个面之间的距离
// 预测抓取姿态
// pcl::PointCloud<pcl::PointXYZRGBNormal>
bool GrsapPosePredict(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud0, 
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
    for(int j=1; j<BoardCloudL.size(); j++)
            viewer->addLine(BoardCloudL[j-1], BoardCloudL[j], 1.0, 1.0, 1.0, "L_border_Line_" + to_string(j));
    viewer->addLine(BoardCloudL[BoardCloudL.size()-1], BoardCloudL[0], 1.0, 1.0, 1.0, "L_border_Line_" + to_string(0));

    for(int j=1; j<BoardCloudR.size(); j++)
            viewer->addLine(BoardCloudR[j-1], BoardCloudR[j], 1.0, 1.0, 1.0, "R_border_Line_" + to_string(j));
    viewer->addLine(BoardCloudR[BoardCloudR.size()-1], BoardCloudR[0], 1.0, 1.0, 1.0, "R_border_Line_" + to_string(0));


    // ==========================
    // 选出最小面，在另外一面生成投影面
    pcl::PointCloud<pcl::PointXYZRGBNormal> BCloud = BoardCloudL.size()<= BoardCloudR.size() ? BoardCloudL : BoardCloudR;
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
            viewer->addLine(ProjectionCloud[j-1], ProjectionCloud[j], 0.0, 0.0, 1.0, "ProjectionCloud_Line_" + to_string(j));
    viewer->addLine(ProjectionCloud[ProjectionCloud.size()-1], ProjectionCloud[0], 0.0, 0.0, 1.0, "ProjectionCloud_Line_" + to_string(0));

    pcl::PointCloud<pcl::PointXYZRGBNormal> CloudRefineProje = CloudRefine(ProjectionCloud);
    pcl::PointCloud<pcl::PointXYZRGBNormal> ACloud = BoardCloudL.size()>= BoardCloudR.size() ? BoardCloudL : BoardCloudR;
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

    // Eigen::Matrix3d DisM1;
    // DisM1(0, 0) = eigenVectorsPCA(0, 1);
    // DisM1(1, 0) = eigenVectorsPCA(1, 1);
    // DisM1(2, 0) = eigenVectorsPCA(2, 1);

    // DisM1(0, 1) = eigenVectorsPCA(0, 0);
    // DisM1(1, 1) = eigenVectorsPCA(1, 0);
    // DisM1(2, 1) = eigenVectorsPCA(2, 0);

    // DisM1(0, 2) = eigenVectorsPCA(0, 2);
    // DisM1(1, 2) = eigenVectorsPCA(1, 2);
    // DisM1(2, 2) = eigenVectorsPCA(2, 2);

    // DisM1.col(0).normalize();
    // DisM1.col(1).normalize();
    // DisM1.col(2) = DisM.col(0).cross(DisM.col(1)); //校正主方向间垂直  叉乘 求法线
	// DisM1.col(1) = DisM.col(2).cross(DisM.col(0));
	// DisM1.col(0) = DisM.col(1).cross(DisM.col(2));

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
    // pcl::PointXYZ ArrowA,ArrowB;
    // ArrowA.x = position_[0]; ArrowA.y = position_[1]; ArrowA.z = position_[2]; 
    // ArrowB.x = ArrowA.x + 0.1*N.normal_x;
    // ArrowB.y = ArrowA.y + 0.1*N.normal_y;
    // ArrowB.z = ArrowA.z + 0.1*N.normal_z;
    // viewer->addArrow<pcl::PointXYZ>(ArrowB, ArrowA, 255, 0, 0, false, "End_arrow");


    std::cout<<"【SUCCESS】 Grasp Predtic! "<<std::endl;
    return true;
}


int main(int argc, char* argv[])
{
    // 读入文件
    pcl::Normal NL;
    double PlaneL;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud0Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    string path0 = "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/normal/10.ply";
    string path1 = "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/normal/11.ply";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(path0, *cloud0Ptr) == -1) {
		PCL_ERROR("Couldn't read file rabbit.pcd\n");
		return(-1);
	}
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(path1, *cloud1Ptr) == -1) {
		PCL_ERROR("Couldn't read file rabbit.pcd\n");
		return(-1);
	}
 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.3);
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud0Ptr, "cloud0");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud0");//设置点云大
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud1Ptr, "cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud1");//设置点云大


    // =================================
    //           正式处理程序部分
    // =================================
    // Normal:0.105843 -0.6507 -0.751922
    // L:0.0482519
    NL.normal_x = 0.105843;
    NL.normal_y = -0.6507;
    NL.normal_z = -0.751922;
    PlaneL = 0.0482519;
    bool suc = GrsapPosePredict(viewer, *cloud0Ptr, *cloud1Ptr, NL, PlaneL);

    // =================================

    while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
    return true;
}