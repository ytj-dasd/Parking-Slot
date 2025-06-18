project(bamboo)
cmake_minimum_required(VERSION 3.8)
find_package(Welkin REQUIRED)

if (WIN32)
    ## /EHsc添加编译选项用于解决boost::expection 问题
    set(CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")
endif(WIN32)

welkin_include(qt pcl opencv osg e57format laszip vtk boost dxf common uface libdxfrw sam flann ceres)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)
add_subdirectory(libs)
add_subdirectory(apps)
add_subdirectory(test)
# add_subdirectory(plugins)
