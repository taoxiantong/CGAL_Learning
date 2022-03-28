# 工程文件解释
- 待整理

## 1. 源文件---执行目标

- **main——CloudMesh**  

  main 链接到CloudMesh,主要完成形状检测部分。  

- **ViewMesh**  
  OFF文件打开和显示。

- **WriteRGB**  
  OFF 文件写入测试。

- **ShapeSmoothing**  
  基于形状的模型平滑操作。  
  两个重要参数：**迭代时间**和**迭代次数**

- **Points_from_vertices**
  从mesh生成点集合。

- CloudMesh.cpp  
  - 该文件读入一个点云文件
  - 三维重建 设置参数
  - 平滑操作 设置参数
  - 表面检测 设置参数
  - 子形状提取
  - 子形状点云提取
  - 子点云计算平均法线
  - 匹配法向对立的两个面
  - 待定


> 两个多边形求交集
> CGAL::intersection()

