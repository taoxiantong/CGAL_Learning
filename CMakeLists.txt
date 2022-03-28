cmake_minimum_required(VERSION 3.1...3.20)
project(CloudMesh)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")

# CGAL and its components
find_package( CGAL REQUIRED OPTIONAL_COMPONENTS Qt5  )

if(CGAL_Qt5_FOUND)
    #required to use basic_viewer
    add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
endif()

if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()


# include helper file
include( ${CGAL_USE_FILE} )


# Boost and its components
#find_package( Boost REQUIRED )

find_package(Boost REQUIRED thread)
add_library(boost INTERFACE IMPORTED)
set_property(TARGET boost PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})

if ( NOT Boost_FOUND )

    message(STATUS "This project requires the Boost library, and will not be compiled.")

    return()

endif()


find_package(Eigen3 3.2.0) #(requires 3.2.0 or greater)
if ( NOT Boost_FOUND )
    message(STATUS "This project requires the Boost library, and will not be compiled.")
    return()
endif()
add_library(Eigen3 INTERFACE IMPORTED)
include(CGAL_Eigen3_support)


find_package(VTK 8 REQUIRED)
find_package(PCL 1.12 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})


set(Ceres_DIR /usr/local/lib/cmake/Ceres)
find_package(Ceres REQUIRED)
include(CGAL_Ceres_support)



# include for local directory
# include for local package

# Creating entries for all C++ files with "main" routine
# ##########################################################

include( CGAL_CreateSingleSourceCGALProgram )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


# add_executable(ShapeDetect src/shape_detect.h src/shape_detect.cpp)
# #        CloudMesh.cpp CloudMesh.h)
# target_link_libraries(ShapeDetect PUBLIC CGAL::Eigen3_support )

# add_executable(ViewMesh src/ViewMesh.cpp)
# target_link_libraries(ViewMesh PUBLIC  CGAL::Eigen3_support )

# add_executable(WriteRGB src/WriteRGB.cpp)
# target_link_libraries(WriteRGB PUBLIC  CGAL::Eigen3_support )

# # 平滑操作 形状平滑以及区域角度平滑  
# if(TARGET CGAL::Ceres_support)
# target_link_libraries(smoothing_plugin PUBLIC CGAL::Ceres_support)
# endif()

# add_executable(MeshSmoothing src/MeshSmoothing.cpp)
# target_link_libraries(MeshSmoothing PUBLIC  CGAL::Eigen3_support CGAL::Ceres_support)

# # 构建带法线的点云集合
# add_executable(Points_from_vertices src/Points_from_vertices.cpp src/Points_from_vertices.h)
# target_link_libraries(Points_from_vertices PUBLIC CGAL::CGAL_Qt5 CGAL::Eigen3_support )


# add_executable(orient_polygon_soup src/orient_polygon_soup_example.cpp)
# target_link_libraries(orient_polygon_soup PUBLIC CGAL::Eigen3_support)


# add_executable(ToolComputer src/ToolComputer.cpp)
# target_link_libraries(ToolComputer PUBLIC CGAL::Eigen3_support ${PCL_LIBRARIES} ${Boost_LIBRARIES} boost_thread)


# add_subdirectory(./src/trimesh_ball_pivoting)


# # 大尺寸重建
# add_executable(scale_space src/scale_space.cpp)
# target_link_libraries(scale_space PUBLIC CGAL::Eigen3_support)


# # 最终重建
# add_executable(reconstruction src/reconstruction.cpp)
# target_link_libraries(reconstruction PUBLIC CGAL::Eigen3_support )


# # 画手指
# add_executable(plot src/plot.cpp)
# target_link_libraries(plot PUBLIC  ${PCL_LIBRARIES} ${Boost_LIBRARIES})

# # # 给出两个面的抓取预测
# add_executable(GraspPredtic src/GraspPredtic.cpp)
# target_link_libraries(GraspPredtic PUBLIC  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} boost_thread)


# 【总程序】，输入网格，表面平滑-》表面检测(save)-》点提取和法线计算(save)
add_executable(CloudMesh src/CloudMesh.cpp src/CloudMesh.h)
target_link_libraries(CloudMesh PUBLIC CGAL::CGAL_Qt5 CGAL::Ceres_support CGAL::Eigen3_support ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} boost_thread)

