#ifndef TRECKBAR_HPP
#define TRECKBAR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h> 
#include <pcl/surface/organized_fast_mesh.h> 
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/common/pca.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <opencv2/opencv.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>
namespace fs = std::filesystem;

typedef pcl::PointCloud<pcl::PointXYZI> PCLI;
typedef pcl::PointCloud<pcl::PointXYZINormal> PCLIN;
typedef pcl::PointCloud<pcl::PointNormal> PCLN;

typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PCLIptr;
typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr PCLINptr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr PCLNptr;

class mesh_struct
{
public:
    PCLIptr cloudXYZI_src, cloudXYZI, cloudXYZI_viz;
    PCLIptr RefXYZI_src, RefXYZI, RefXYZI_viz;
    PCLIptr OriginXYZI_src, OriginXYZI, OriginXYZI_viz;
    PCLINptr cloudXYZIN, RefXYZIN, OriginXYZIN;
    pcl::PolygonMesh::Ptr mesh, ref_mesh, origin_mesh;

private:
    float DegreesToRadians(float degrees);
    float RadiansToDegrees(float rad);

public:
    mesh_struct(std::string filename);
    ~mesh_struct();
    void Load_cloud_from_file(std::string filename);
    void Load_cloud_from_file_currection(std::string filename);
    void Write_cloud_to_file(std::string filename);
    PCLIptr PassThroughFilter(PCLIptr input_cloud);
    PCLIptr XYZSelection(PCLIptr input_cloud);
    PCLIptr SmoothByIntensity(PCLIptr input_cloud);
    PCLIptr SorFilter(PCLIptr input_cloud);
    PCLIptr VoxelGridFilter(PCLIptr input_cloud);
    PCLINptr VoxelGridFilterNorm(PCLINptr input_cloud);
    PCLINptr Smoothing(PCLIptr input_cloud);
    pcl::PolygonMesh::Ptr Meshing(PCLINptr input_cloud);
    PCLIptr convertToXYZI(const PCLINptr &input_cloud);
    PCLINptr computeNormals(PCLIptr cloud);
    pcl::PolygonMesh::Ptr Poisson_mesh(PCLINptr input_cloud);    
    pcl::PolygonMesh::Ptr FastMesh(PCLINptr input_cloud);
    pcl::PolygonMesh::Ptr Alpha_shapes(PCLINptr input_cloud);
    pcl::PolygonMesh::Ptr MarchingCubes(PCLINptr input_cloud);
    void write_point_xyz(pcl::visualization::PCLVisualizer* viewer);
    void fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    
    PCLIptr generatePlane(std::vector<float> coef, float range);
    std::vector<float> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    float count_normal_error(std::vector<float> coef, PCLINptr input_cloud);
};

class treckbar
{
public:
    std::vector<std::string> varnames; 
    bool treckbar_flag;

public:
    treckbar()
    {
        cv::namedWindow("Controls", cv::WINDOW_NORMAL);
        treckbar_flag = 1;
    }

    void push_treckbar(std::string name, int* number, int max_num)
    {
        auto it = std::find(varnames.begin(), varnames.end(), name);
        if (it == varnames.end())
        {
            varnames.push_back(name);
            cv::createTrackbar(name, "Controls", number, max_num, on_trackbar, this);
        }
    }

    static void on_trackbar(int, void* userdata) 
    {
        treckbar* self = static_cast<treckbar*>(userdata);
        self->treckbar_flag = 1;
    }
};

extern treckbar global_treckbar;
#endif // TRECKBAR_HPP