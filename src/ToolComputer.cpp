//
// Created by txt on 2021/12/5.
//


// 
//  给出A B 两个点 的位置（x，y，z） 以及法线，两个点点法线相反
//  求出以A 点为圆心旋转的的抓取法线，垂直与A 的法线。AB 距离为抓取距离。
//  目前问题： 数值求解不稳定
// 


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


// PCL inlludes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <boost/thread/thread.hpp>
#include <boost/format.hpp>


using ShapeNormal = std::pair<pcl::PointXYZ, pcl::Normal>;
using namespace std;

int main(){
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    
    ShapeNormal A;
    A.first.x = 0;
    A.first.y = 0;
    A.first.z = 0;
    A.second.normal_x = 0.5;
    A.second.normal_y = 0.5;
    A.second.normal_z = pow(0.5,0.5);

    ShapeNormal B;
    B.first.x = -0.5;
    B.first.y = -0.5;
    B.first.z = -pow(0.5,0.5);
    B.second.normal_x = -0.5;
    B.second.normal_y = -0.5;
    B.second.normal_z = -pow(0.5,0.5);

    // 以A 点为基 求出Z=0 的法线
    double Cx = A.first.x, Cy = A.first.y, Cz= A.first.z;
    double Na = A.second.normal_x, Nb = A.second.normal_y, Nc = A.second.normal_z;
    double x1,y1,z1;  // 目标法线
    // 设Nz=0
    z1 = 0.0;
    double a = Na, b = Nb, c = Nc;

    // Nx = -(b/a) * ( (a^2) / (a^2+b^2) )^0.5
    double Nxtemp;
    try
    {
        double temp0 = pow(b,2)+pow(a,2);
        Nxtemp = pow((pow((2*a*c*z1), 2) - 4*(pow(b,2)+pow(a,2))*(pow(c*z1,2)+pow(z1*b,2)-pow(b,2)) ), 0.5);
        x1 = (-2*a*c*z1 + Nxtemp) / 2*temp0;
        y1 = -(c*z1+a*x1)/b;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    pcl::PointXYZ AEnd, BEnd;
    double scale = 1;
    AEnd.x = A.first.x + scale * x1;
    AEnd.y = A.first.y + scale * y1;
    AEnd.z = A.first.z + scale * z1;

    BEnd.x = B.first.x + scale * x1;
    BEnd.y = B.first.y + scale * y1;
    BEnd.z = B.first.z + scale * z1;
    

    // baes Line
    pcl::PointXYZ base;
    base.x = 5 * A.second.normal_x;
    base.y = 5 * A.second.normal_y;
    base.z = 5 * A.second.normal_z;
    viewer->addLine<pcl::PointXYZ>(A.first, base, 255, 255, 255, "base");


    viewer->addLine<pcl::PointXYZ>(A.first, AEnd, 255, 0, 0, "A_E");
    viewer->addLine<pcl::PointXYZ>(B.first, BEnd, 255, 0, 0, "B_E");
    viewer->addLine<pcl::PointXYZ>(AEnd,    BEnd, 0, 0, 255, "AE_BE");

    pcl::PointXYZ PAStart(A.first);
    pcl::PointXYZ PBStart(B.first);


    cout<<"(a,b,c): "<< a << " "<< b << " "<< c <<endl;
    cout<<"(x1,y1,z1): "<< x1 << " "<< y1 << " "<< z1 <<endl;
    for (size_t i = 1; i <= 180; i++)
    {
        try
        {
            cout << "solve: " << i*30 <<endl;

            double theta = i * M_PI/90;
            double lambd = cos(theta);
            
            double temp1 = (c*x1 - a*z1) / (c*z1);
            double temp2 = (b*x1 - a*y1) / (b*y1);

            double D = (c*x1 - a*z1);
            double G = (b*x1 - a*y1);

            double E = pow((b*z1-c*y1) ,2);
            double F = pow((c*y1-b*z1) ,2);

            double A = E*F + F*pow(D,2) + E*pow(G,2);
            double B = 2*F*D*c*lambd + 2*E*G*b*lambd;
            double C = F*pow((c*lambd),2) + E*pow((b*lambd),2) - E*F;

            double temp3 = pow((pow(B,2)-4*A*C), 0.5);
            double x2  = (-B+temp3)/(2*A);
            double x2_ = (-B-temp3)/(2*A);

            double y2  = (x2 * (c*x1-a*z1) + c*lambd) / (b*z1 - c*y1);
            double y2_ = (x2_ * (c*x1-a*z1) + c*lambd) / (b*z1 - c*y1);

            double z2  = (x2 * (b*x1-a*y1) + b*lambd) / (c*y1 - b*z1);
            double z2_ = (x2_ * (b*x1-a*y1) + b*lambd) / (c*y1 - b*z1);


            cout<<"sovle1_(x2,y2,z2): "<< x2 << " "<< y2 << " "<< z2 <<endl;
            cout<<"sovle2_(x2,y2,z2): "<< x2_ << " "<< y2_ << " "<< z2_ <<endl;


            // 显示
            pcl::PointXYZ AEnd, BEnd;
            double scale = 1;
            AEnd.x = PAStart.x + scale * x2;
            AEnd.y = PAStart.y + scale * y2;
            AEnd.z = PAStart.z + scale * z2;

            BEnd.x = PBStart.x + scale * x2;
            BEnd.y = PBStart.y + scale * y2;
            BEnd.z = PBStart.z + scale * z2;

            std::string str1 = std::to_string(i)+"A_E";
            string str2 = std::to_string(i)+"B_E";
            string str3 = std::to_string(i)+"AE_BE";
            viewer->addLine<pcl::PointXYZ>(PAStart, AEnd, 255, 0, 0, str1);
            viewer->addLine<pcl::PointXYZ>(PBStart, BEnd, 255, 0, 0, str2);
            viewer->addLine<pcl::PointXYZ>(AEnd,    BEnd, 0, 255, 0, str3);
            viewer->spinOnce(1500);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        

    }

    while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
    return 0;
}