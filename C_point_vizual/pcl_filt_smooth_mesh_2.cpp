#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

int main() {
    std::string input_file = "../points_and_meshes/pointcloud_log_4_edit.txt";
    std::ifstream inputFile(input_file);
    if (!inputFile.is_open()) {
        std::cerr << "Не удалось открыть файл input.txt" << std::endl;
        return 1;
    }


    // Step 1: Load points from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string line;
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        std::vector<float> numbers;
        float num;
        while (iss >> num) {
            numbers.push_back(num);
        }
        if (numbers.size() >= 3) {
            cloud->points.push_back(pcl::PointXYZ(numbers[0], numbers[1], numbers[2]));
        }
    }
    inputFile.close();

    // Step 2: Downsample using Voxel Grid Filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // Adjust the leaf size as needed //0.1
    voxel_filter.filter(*cloud_filtered);
    // Save the filtered point cloud
    pcl::io::savePCDFileASCII("../points_and_meshes/filtered_points.pcd", *cloud_filtered);
    std::cout << "Downsampled and smoothed point cloud saved as 'filtered_points.pcd'" << std::endl;



    // Step 3: Smooth using Moving Least Squares
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;  // Updated to output PointNormal
    mls.setInputCloud(cloud_filtered);
    mls.setSearchRadius(0.2);  // Set the smoothing radius
    mls.setPolynomialOrder(2);  
    mls.setComputeNormals(true);  // Enable normal computation (needed for meshing)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    mls.setSearchMethod(tree);
    mls.process(*cloud_smoothed_with_normals);

    // Save the smoothed point cloud
    pcl::io::savePCDFileASCII("../points_and_meshes/smoothed_points.pcd", *cloud_smoothed_with_normals);
    std::cout << "Downsampled and smoothed point cloud saved as 'smoothed_points.pcd'" << std::endl;



    // Step 4: Create a mesh using Greedy Projection Triangulation
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_smoothed_with_normals);
    // Set the parameters
    gp3.setSearchRadius(0.5);  // Set the maximum distance between connected points (adjust as needed) 0.25
    gp3.setMu(2.5);  // Multiplicative factor for search radius 2.5
    gp3.setMaximumNearestNeighbors(100);  // Limit the number of nearest neighbors //100
    gp3.setMaximumSurfaceAngle(M_PI / 1.2);  // Maximum angle (in radians) between surface normals // pi/4
    gp3.setMinimumAngle(M_PI / 72);  // Minimum angle for a triangle // pi/18
    gp3.setMaximumAngle(2 * M_PI / 1.2);  // Maximum angle for a triangle // 2*pi/3
    gp3.setNormalConsistency(false);

    // Generate the mesh
    gp3.setInputCloud(cloud_smoothed_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(mesh);

    // Save the mesh to a file
    pcl::io::savePLYFile("../points_and_meshes/mesh_smooth.ply", mesh);
    std::cout << "Mesh saved as 'smoothed_mesh.ply'" << std::endl;

    return 0;
}