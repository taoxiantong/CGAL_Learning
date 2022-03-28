/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004-2016                                           \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/
#include<vcg/complex/complex.h>
#include <vcg/complex/algorithms/create/ball_pivoting.h>

// input output
#include <wrap/io_trimesh/import_ply.h>
#include <wrap/io_trimesh/export_ply.h>

#include <vcg/complex/algorithms/hole.h>

using namespace vcg;
using namespace std;

class MyFace;
class MyVertex;

struct MyUsedTypes : public UsedTypes<	Use<MyVertex>		::AsVertexType,
                                                                                Use<MyFace>			::AsFaceType>{};

class MyVertex  : public Vertex< MyUsedTypes, vertex::Coord3f, vertex::Normal3f, vertex::BitFlags, vertex::Mark>{};
class MyFace    : public Face  < MyUsedTypes, face::VertexRef, face::Normal3f, face::BitFlags > {};
class MyMesh    : public vcg::tri::TriMesh< vector<MyVertex>, vector<MyFace> > {};

bool callback(int percent, const char *str) {
  cout << "str: " << str << " " << percent << "%\n";
  return true;
}
bool callback1(int percent, const char *str) {
  cout << "str: " << str << " " << percent << "%\n";
  return true;
}

int  main(int argc, char **argv)
{
 if(argc<3)
    {
        printf(
      "Usage: trimesh_ball_pivoting filein.ply fileout.ply [opt]\n"
      "options: \n"
      "-r <val> radius of the rolling ball\n"
      "-c <val> clustering radius (as fraction of radius) default: 0.05\n"
            );
        exit(0);
    }

   float radius = 0.0f;
   float clustering = 0.02;
   double CreaseThr = 90.0;
   int i = 3;
   double flag = 0.0;
    while(i<argc)
        {
            if(argv[i][0]!='-')
                {printf("Error unable to parse option '%s'\n",argv[i]); exit(0);}
            switch(argv[i][1])
            {
                case 'r' :	radius = atof(argv[++i]); printf("Using %f sphere radius\n",radius);  break;
                case 'c' :	clustering = atof(argv[++i]); printf("Using %f clustering radius\n",clustering); break;
                case 't' :	CreaseThr = atof(argv[++i]); printf("Using %f CreaseThr radius\n",CreaseThr); break;
                case 'f' :	flag = atof(argv[++i]); printf("Using %f flag radius\n",flag); break;

                default : {printf("Error unable to parse option '%s'\n",argv[i]); exit(0);}
            }
            ++i;
        }
    if(radius == 0)
      printf("Autodetecting ball radius...\n");

    MyMesh m;

    if(vcg::tri::io::ImporterPLY<MyMesh>::Open(m,argv[1])!=0)
    {
      printf("Error reading file  %s\n",argv[1]);
      exit(0);
    }
    vcg::tri::Allocator<MyMesh>::CompactEveryVector(m);
    vcg::tri::UpdateBounding<MyMesh>::Box(m);
    vcg::tri::UpdateNormal<MyMesh>::PerFace(m);
    printf("Input mesh  vn:%i fn:%i\n",m.VN(),m.FN());

  int t0=clock();
  // Initialization
  tri::BallPivoting<MyMesh> pivot(m, radius, clustering, math::ToRad(CreaseThr));
  printf("Ball radius: %f\nClustering points withing %f radii\n", pivot.radius, clustering);

  int t1=clock();
  // the main processing
  pivot.BuildMesh(callback);

  int t2=clock();

  printf("Output mesh vn:%i fn:%i\n",m.VN(),m.FN());
  printf("Created in :%i msec (%i+%i)\n",t2-t0,t1-t0,t2-t1);



  // Holes Fill
  if (flag>=1.0)
  {
		// if (  tri::Clean<MyMesh>::CountNonManifoldEdgeFF(m) > 0){
		// 	printf("Mesh has some not 2-manifold edges, filter requires edge manifoldness");
		// }
    std::cout<<"ari 0"<<std::endl;
		size_t OriginalSize= m.face.size();
		int MaxHoleSize = 30;
		bool SelectedFlag = false;
		bool SelfIntersectionFlag = true;
		bool NewFaceSelectedFlag = true;
		int holeCnt;
    std::cout<<"ari 1"<<std::endl;
		if( SelfIntersectionFlag )
		{ 
      holeCnt = tri::Hole<MyMesh>::EarCuttingIntersectionFill<tri::SelfIntersectionEar<MyMesh>>(m, MaxHoleSize, SelectedFlag, callback1);
      std::cout<<"ari 2"<<std::endl;
    }
		else
			holeCnt = tri::Hole<MyMesh>::EarCuttingFill<vcg::tri::MinimumWeightEar<MyMesh>>(m,MaxHoleSize,SelectedFlag, callback1);
		printf("Closed %i holes and added %i new faces",holeCnt, m.fn-OriginalSize);
    std::cout<<"ari 3"<<std::endl;

		// hole filling filter does not correctly update the border flags (but the topology is still ok!)
		if(NewFaceSelectedFlag)
		{
      std::cout<<"ari 3"<<std::endl;
			tri::UpdateSelection<MyMesh>::FaceClear(m);
			for(size_t i=OriginalSize;i<m.face.size();++i)
				if(!m.face[i].IsD()) m.face[i].SetS();
		}

  }
  
  vcg::tri::io::ExporterPLY<MyMesh>::Save(m,argv[2]);
  return 0;

}
