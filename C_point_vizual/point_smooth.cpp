#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

int main() {
    std::string input_file = "../points_and_meshes/pointcloud_log_3.txt";
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
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // Adjust the leaf size as needed
    voxel_filter.filter(*cloud_filtered);

    // Step 3: Smooth using Moving Least Squares
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(cloud_filtered);
    mls.setSearchRadius(0.2);  // Set the smoothing radius
    mls.setPolynomialOrder(2);  // Enable polynomial fitting
    mls.setComputeNormals(false);  // Disable normal computation
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    mls.setSearchMethod(tree);
    mls.process(*cloud_smoothed);

    // Save the smoothed and downsampled cloud (optional)
    pcl::io::savePCDFileASCII("smoothed_downsampled_cloud.pcd", *cloud_smoothed);
    std::cout << "Downsampled and smoothed point cloud saved as 'smoothed_downsampled_cloud.pcd'" << std::endl;

    return 0;
}