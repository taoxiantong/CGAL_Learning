//
// Created by txt on 2021/12/5.
//

#include "CloudMesh.h"

int main(int argc, char * argv[]){
    
    const char* filename = (argc>1) ? argv[1] : "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/data/polygon_mesh.off";
    string str(filename);

    // 1. 定义模型，初始化
    CloudMesh MyModel(str);
    MyModel.draw(MyModel.MeshInput);

    // 2. 平滑模型
    SMesh smoothM = MyModel.Smoothing(MyModel.MeshInput, 1, 0.00001);
    // smoothM = MyModel.Smoothing(smoothM, 5, 0.00001);
    MyModel.draw(smoothM);

    // 3. 表面检测提取
    Bbox box =  MyModel.ComputerBbox(smoothM);
    double Mdix = 0.01 * sqrt(pow((box.xmax()-box.xmin()),2)+pow((box.ymax()-box.ymin()),2)+pow((box.zmax()-box.zmin()),2));
    int angle = 30;
    int Msize = int (0.01 * smoothM.num_vertices());
    cout<<"Mesh Bbox: "<< 100 * Mdix<<endl;
    vector<vector<uint32_t>> shape_face_indexs = MyModel.ShapeDetect(smoothM, Mdix, angle, Msize);
    cout<<"shape size: "<< shape_face_indexs.size()<<endl;

    // Point_set MyPointsSet = MyModel.PointSetFromVertices(smoothM);

    // 4. 提取出形状ID 生成子点云集
    vector<PCLCloud> clouds;
    srand(static_cast<unsigned int>(time(nullptr)));
    for(int i=0; i<shape_face_indexs.size(); i++){
        Point_set subM = MyModel.SubMeshCreatFromShape(smoothM, shape_face_indexs[i]);

        // 生成随机颜色
        CGAL::Random rnd(static_cast<unsigned int>(i));
        const Color color(
                static_cast<unsigned char>( rnd.get_int(0, 255)),
                static_cast<unsigned char>( rnd.get_int(0, 255)),
                static_cast<unsigned char>( rnd.get_int(0, 255)));

        PCLCloud cloud = MyModel.SubPCLCloudCreatFromShape(subM, color);
        clouds.push_back(cloud);
    }


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(Mdix * 25);

    // 定一个集 保存平面中点和法线
    vector<ShapeNormal> SN_Set; 

    // 5. 计算每个平面中心和法向量  &&& 显示
    // const int di = (argc>2) ? atoi(argv[2]) : 1 ;
    // for(int i=di; i<=di; i++){
    for(int i=0; i<clouds.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::copyPointCloud(clouds[i], *cloudptr);

        // 计算法向量
        cout<<"【Plane " << i << " 】"<<endl;
        ShapeNormal SN = MyModel.ShapeNormalAver(clouds[i]);
        // 保存平面信息
        SN_Set.push_back(SN);

        // string savestr = "/home/txt/taoxiantong/taoxiantong/code/CloudMesh/normal/" + to_string(i) + ".txt";
        // MyModel.SaveNormalAngle(clouds[i], savestr );

        // 显示面中心和法向量
        string c = "center_" + to_string(i);
        pcl::PointXYZ arrowend;
        double scale = Mdix *10;

        // 显示法线箭头，和平面质心点
        arrowend.y = SN.first.y + scale * SN.second.normal_y;
        arrowend.x = SN.first.x + scale * SN.second.normal_x;
        arrowend.z = SN.first.z + scale * SN.second.normal_z;
        cout<<"【Plane Normal】: ("<< SN.second.normal_x<<", "<< SN.second.normal_y<<", "<< SN.second.normal_z<<")"<<endl<<endl;

        // viewer->addSphere(SN.first, 0.005, c);
        // viewer->addArrow<pcl::PointXYZ>(arrowend, SN.first, 255, 0, 0, false, (c+"arrow"));


        // 显示点云
        string str_p = "cloud_" + to_string(i);
        // pcl::copyPointCloud(clouds[i], *cloudptr);
        viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloudptr, str_p);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, str_p);//设置点云大

        // 显示 点云法线
        // viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(cloudptr, 1, 2, (str_p+"normal"));

        viewer->spinOnce(100);
        // viewer->removeAllPointClouds();
        // viewer->removeAllShapes();
    }

    // 查找匹配面
    vector<vector<int> > InvShapeSet = MyModel.MatchPlane(SN_Set);
    cout<<"InvShapeSet Size: " << InvShapeSet.size() <<endl;
    srand(static_cast<unsigned int>(time(nullptr)));
    for (size_t i = 0; i < InvShapeSet.size(); i++)
    {
        CGAL::Random rnd(static_cast<unsigned int>(i));
        const Color color(
                static_cast<unsigned char>(25 + rnd.get_int(0, 220)),
                static_cast<unsigned char>(25 + rnd.get_int(0, 220)),
                static_cast<unsigned char>(25 + rnd.get_int(0, 220)));

        // 显示法线箭头，和平面质心点
        double scale = Mdix *10;
        pcl::PointXYZ arrowend;
        arrowend.x = SN_Set[InvShapeSet[i][0]].first.x + scale * SN_Set[InvShapeSet[i][0]].second.normal_x;
        arrowend.y = SN_Set[InvShapeSet[i][0]].first.y + scale * SN_Set[InvShapeSet[i][0]].second.normal_y;
        arrowend.z = SN_Set[InvShapeSet[i][0]].first.z + scale * SN_Set[InvShapeSet[i][0]].second.normal_z;
        viewer->addArrow<pcl::PointXYZ>(arrowend, SN_Set[InvShapeSet[i][0]].first, color.r()/255.0, color.g()/255.0, color.b()/255.0, false, (to_string(i)+"arrow"));

        pcl::PointXYZ arrowend_;
        arrowend_.x = SN_Set[InvShapeSet[i][1]].first.x + scale * SN_Set[InvShapeSet[i][1]].second.normal_x;
        arrowend_.y = SN_Set[InvShapeSet[i][1]].first.y + scale * SN_Set[InvShapeSet[i][1]].second.normal_y;
        arrowend_.z = SN_Set[InvShapeSet[i][1]].first.z + scale * SN_Set[InvShapeSet[i][1]].second.normal_z;
        viewer->addArrow<pcl::PointXYZ>(arrowend_, SN_Set[InvShapeSet[i][1]].first, color.r()/255.0, color.g()/255.0, color.b()/255.0, false, (to_string(i)+"arrow_"));

        cout<<"(i,j)" << InvShapeSet[i][0] << " " << InvShapeSet[i][1] <<endl;
    }
    


    // 显示
    // viewer->setBackgroundColor(255, 255, 255);

    // for(int i=0; i<clouds.size(); i++){
    //     string s = "cloud_" + to_string(i);
    //     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //     // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    //     pcl::copyPointCloud(clouds[i], *cloudptr);
    //     cout<<"cloud size: "<< cloudptr->size()<<endl;
    //     viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloudptr, s);
    //     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, s);//设置点云大
    //     viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(cloudptr, 1, 1, (s+"normal"));
    //     // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.01, "normals");
    //     viewer->spinOnce(1000);
    // }

    while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
    // MyModel.draw(subM);
    // MyModel.draw(smoothM);
    return true;
}
