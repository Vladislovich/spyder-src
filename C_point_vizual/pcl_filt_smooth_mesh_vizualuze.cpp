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

#include <opencv2/opencv.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
/*
10
35
2
50
50
100
12
72
12
*/

int number_of_neighbors_int = 20; 
int distance_to_neighbors_int = 14; 

int LeafSize_int = 10; 

int SearchRadius_int = 35;
int PolynomialOrder_int = 2;

int MeshSearchRadius_int = 50;
int Mu_int = 50;
int MaximumNearestNeighbors_int = 100;

int MaximumSurfaceAngle_int = 12;
int MinimumAngle_int = 72;
int MaximumAngle_int = 12;

bool treckbar_flag = 1;

int points_flag = 0;
int mesh_flag = 1;

void on_trackbar(int, void*) {
    treckbar_flag = 1;
}

void create_trackbar()
{
    // Создайте окно для трекбаров
    cv::namedWindow("Controls", cv::WINDOW_NORMAL);
    // Добавьте трекбары для настройки цвета фона

    cv::createTrackbar("sor_filter:number_of_neighbors", "Controls", &number_of_neighbors_int, 100, on_trackbar);
    cv::createTrackbar("sor_filter:distance_to_neighbors", "Controls", &distance_to_neighbors_int, 100, on_trackbar);

    cv::createTrackbar("Voxel_Grid_Filter:LeafSize", "Controls", &LeafSize_int, 100, on_trackbar);

    cv::createTrackbar("Smoothing:SearchRadius", "Controls", &SearchRadius_int, 100, on_trackbar);
    cv::createTrackbar("Smoothing:PolynomialOrder", "Controls", &PolynomialOrder_int, 5, on_trackbar);

    cv::createTrackbar("Meshing:MeshSearchRadius", "Controls", &MeshSearchRadius_int, 200, on_trackbar);
    cv::createTrackbar("Meshing:Mu", "Controls", &Mu_int, 500, on_trackbar);
    cv::createTrackbar("Meshing:MaximumNearestNeighbors", "Controls", &MaximumNearestNeighbors_int, 500, on_trackbar);

    cv::createTrackbar("Meshing:MaximumSurfaceAngle", "Controls", &MaximumSurfaceAngle_int, 100, on_trackbar);
    cv::createTrackbar("Meshing:MinimumAngle", "Controls", &MinimumAngle_int, 100, on_trackbar);
    cv::createTrackbar("Meshing:MaximumAngle", "Controls", &MaximumAngle_int, 100, on_trackbar);

    cv::createTrackbar("points_flag", "Controls", &points_flag, 3, on_trackbar);
    cv::createTrackbar("mesh_flag", "Controls", &mesh_flag, 1, on_trackbar);
    
}




pcl::PointCloud<pcl::PointXYZ>::Ptr Load_cloud_from_file()
{
    std::string input_file = "../points_and_meshes/pointcloud_log_5_edit.txt";
    std::ifstream inputFile(input_file);
    if (!inputFile.is_open()) {
        std::cerr << "Не удалось открыть файл input.txt" << std::endl;
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

    return cloud;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr del_neighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
    sor_filter.setInputCloud(cloud);
    sor_filter.setMeanK(number_of_neighbors_int); // Количество соседей для анализа (например, 50)//20
    sor_filter.setStddevMulThresh((double)distance_to_neighbors_int / 100); // Удаляет точки, которые находятся дальше чем 1 стандартное отклонение от среднего расстояния //1.4
    sor_filter.filter(*sor_cloud);

    return sor_cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Voxel_Grid_Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Step 2: Downsample using Voxel Grid Filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize((float)LeafSize_int / 100, (float)LeafSize_int / 100, (float)LeafSize_int / 100);  // Adjust the leaf size as needed //0.1
    voxel_filter.filter(*cloud_filtered);
    
    // Save the filtered point cloud
    //pcl::io::savePCDFileASCII("../points_and_meshes/filtered_points.pcd", *cloud_filtered);
    //std::cout << "Downsampled and smoothed point cloud saved as 'filtered_points.pcd'" << std::endl;

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointNormal>::Ptr Smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    // Step 3: Smooth using Moving Least Squares
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;  // Updated to output PointNormal
    mls.setInputCloud(cloud_filtered);
    mls.setSearchRadius((double)SearchRadius_int / 100);  // Set the smoothing radius //0.2
    mls.setPolynomialOrder(PolynomialOrder_int);  //2
    mls.setComputeNormals(true);  // Enable normal computation (needed for meshing)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    mls.setSearchMethod(tree);
    mls.process(*cloud_smoothed_with_normals);

    // Save the smoothed point cloud
    //pcl::io::savePCDFileASCII("../points_and_meshes/smoothed_points.pcd", *cloud_smoothed_with_normals);
    //std::cout << "Downsampled and smoothed point cloud saved as 'smoothed_points.pcd'" << std::endl;

    return cloud_smoothed_with_normals;
}


pcl::PolygonMesh Meshing(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_with_normals)
{

    // Step 4: Create a mesh using Greedy Projection Triangulation
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_smoothed_with_normals);
    // Set the parameters
    gp3.setSearchRadius((double)MeshSearchRadius_int / 100);  // Set the maximum distance between connected points (adjust as needed) 0.5
    gp3.setMu((double)Mu_int / 10);  // Multiplicative factor for search radius 2.5
    gp3.setMaximumNearestNeighbors(MaximumNearestNeighbors_int);  // Limit the number of nearest neighbors //100
    
    gp3.setMaximumSurfaceAngle(M_PI / ((double)MaximumSurfaceAngle_int / 10));  // Maximum angle (in radians) between surface normals // pi/4
    gp3.setMinimumAngle(M_PI / (double)MinimumAngle_int);  // Minimum angle for a triangle // pi/18
    gp3.setMaximumAngle(2 * M_PI / ((double)MaximumAngle_int / 10));  // Maximum angle for a triangle // 2*pi/3
    gp3.setNormalConsistency(false);

    // Generate the mesh
    gp3.setInputCloud(cloud_smoothed_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(mesh);

    // Save the mesh to a file
    //pcl::io::savePLYFile("../points_and_meshes/mesh_smooth.ply", mesh);
    //std::cout << "Mesh saved as 'smoothed_mesh.ply'" << std::endl;

    return mesh;

}

int main() {  
    create_trackbar();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = Load_cloud_from_file();

    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PolygonMesh mesh; 

    // Создайте визуализатор и настройте его
    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.setBackgroundColor(0.2, 0.2, 0.2); // Черный фон
    viewer.addCoordinateSystem(0.2); // Добавить систему координат
    viewer.initCameraParameters();

    // Цикл визуализации
    while (!viewer.wasStopped()) { //q       
        int key = cv::waitKey(1); 
        
        if (treckbar_flag == 1) //a
        {
            treckbar_flag = 0;

            viewer.removePointCloud("sor_cloud");
            viewer.removePointCloud("cloud_filtered");
            viewer.removePolygonMesh("mesh");

            sor_cloud = del_neighbors(cloud);
            cloud_filtered = Voxel_Grid_Filter(sor_cloud);
            cloud_smoothed_with_normals = Smoothing(cloud_filtered);

            mesh = Meshing(cloud_smoothed_with_normals);
            

            if (points_flag == 0)
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sor_cloud_Color(sor_cloud, 255,255,0);
                viewer.addPointCloud(sor_cloud, sor_cloud_Color, "sor_cloud");
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sor_cloud");
            }

            if (points_flag == 1)
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_filtered_Color(cloud_filtered, 0,255,255);
                viewer.addPointCloud(cloud_filtered, cloud_filtered_Color, "cloud_filtered");
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_filtered");
            }

            if (mesh_flag == 1)
                viewer.addPolygonMesh(mesh, "mesh");
        }
        viewer.spinOnce(100); 
    }

    return 0;
}
