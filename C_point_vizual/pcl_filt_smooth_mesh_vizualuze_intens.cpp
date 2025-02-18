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

#include <opencv2/opencv.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
/* 10 35 2 50 50 100 12 72 12 */
/*10 100 7 70 6 12 2 50 50 100 12 72 12 3 0*/
/*10 100 8 70 4 12 2 53 59 50 10 4 10 10 5*/

int search_radius_int = 2;

int low_intens_int = 10;
int high_intens_int = 100;

int number_of_neighbors_int = 7; 
int distance_to_neighbors_int = 70; 

int LeafSize_int = 6; 

int SearchRadius_int = 12;
int PolynomialOrder_int = 2;

int LeafSize2_int = 5; 

int MeshSearchRadius_int = 50;
int Mu_int = 100;
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

    cv::createTrackbar("SmoothByIntensity: search_radius", "Controls", &search_radius_int, 6, on_trackbar);

    cv::createTrackbar("PassThroughFilter: low_intens", "Controls", &low_intens_int, 100, on_trackbar);
    cv::createTrackbar("PassThroughFilter: high_intens", "Controls", &high_intens_int, 100, on_trackbar);

    cv::createTrackbar("SorFilter: number_of_neighbors", "Controls", &number_of_neighbors_int, 100, on_trackbar);
    cv::createTrackbar("SorFilter: distance_to_neighbors", "Controls", &distance_to_neighbors_int, 100, on_trackbar);

    cv::createTrackbar("VoxelGridFilter: LeafSize", "Controls", &LeafSize_int, 100, on_trackbar);

    cv::createTrackbar("Smoothing: SearchRadius", "Controls", &SearchRadius_int, 100, on_trackbar);
    cv::createTrackbar("Smoothing: PolynomialOrder", "Controls", &PolynomialOrder_int, 5, on_trackbar);

    cv::createTrackbar("Meshing: MeshSearchRadius", "Controls", &MeshSearchRadius_int, 200, on_trackbar);
    cv::createTrackbar("Meshing: Mu", "Controls", &Mu_int, 500, on_trackbar);
    cv::createTrackbar("Meshing: MaximumNearestNeighbors", "Controls", &MaximumNearestNeighbors_int, 500, on_trackbar);

    cv::createTrackbar("Meshing: MaximumSurfaceAngle", "Controls", &MaximumSurfaceAngle_int, 100, on_trackbar);
    cv::createTrackbar("Meshing: MinimumAngle", "Controls", &MinimumAngle_int, 100, on_trackbar);
    cv::createTrackbar("Meshing: MaximumAngle", "Controls", &MaximumAngle_int, 100, on_trackbar);

    cv::createTrackbar("VoxelGridFilter_2: LeafSize", "Controls", &LeafSize2_int, 100, on_trackbar);

    cv::createTrackbar("points_flag", "Controls", &points_flag, 10, on_trackbar);
    cv::createTrackbar("mesh_flag", "Controls", &mesh_flag, 1, on_trackbar);    
}


pcl::PointCloud<pcl::PointXYZI>::Ptr Load_cloud_from_file()
{
    std::string input_file = "../points_and_meshes/pointcloud_log_5_edit.txt";
    std::ifstream inputFile(input_file);
    if (!inputFile.is_open()) {
        std::cerr << "Не удалось открыть файл input.txt" << std::endl;
    }


    // Step 1: Load points from file
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::string line;
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        std::vector<float> numbers;
        float num;
        while (iss >> num) {
            numbers.push_back(num);
        }
        if (numbers.size() >= 4) {
            pcl::PointXYZI point;
            point.x = numbers[0];
            point.y = numbers[1];
            point.z = numbers[2];
            point.intensity = numbers[3];
            cloud->points.push_back(point);
        }
    }
    inputFile.close();

    return cloud;
}




pcl::PointCloud<pcl::PointXYZI>::Ptr SmoothByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud) 
{ //сглаживающий фильтр учитывающий интенсивности
    pcl::PointCloud<pcl::PointXYZI>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);

    // Create a KD-tree representation of the input point cloud
    tree->setInputCloud(input_cloud);

    // Create a MovingLeastSquares object
    pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls;
    mls.setInputCloud(input_cloud);
    mls.setSearchMethod(tree);
    mls.setSearchRadius((double)search_radius_int / 10);
    mls.setPolynomialOrder(2);
    mls.setComputeNormals(false);

    // Perform the smoothing
    mls.process(*smoothed_cloud);

    return smoothed_cloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr PassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
    //фильтр по интенсивностям
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass_filter;
    pass_filter.setInputCloud(input_cloud);
    pass_filter.setFilterFieldName("intensity"); // Фильтрация по полю интенсивности
    pass_filter.setFilterLimits((float)low_intens_int, (float)high_intens_int); // Удаление точек с интенсивностью менее 0.2 и более 1.0
    pass_filter.filter(*output_cloud);

    return output_cloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr SorFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{ //фильтр удаляющий точки в зависимости от количества соседей на расстоянии от точки
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;
    sor_filter.setInputCloud(input_cloud);
    sor_filter.setMeanK(number_of_neighbors_int); // Количество соседей для анализа (например, 50)//20
    sor_filter.setStddevMulThresh((double)distance_to_neighbors_int / 100); // Удаляет точки, которые находятся дальше чем 1 стандартное отклонение от среднего расстояния //1.4
    sor_filter.filter(*output_cloud);

    return output_cloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr VoxelGridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
    //замена точек в вокселе одной точкой 
    // Step 2: Downsample using Voxel Grid Filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize((float)LeafSize_int / 100, (float)LeafSize_int / 100, (float)LeafSize_int / 100);  // Adjust the leaf size as needed //0.1
    voxel_filter.filter(*output_cloud);

    return output_cloud;
}


pcl::PointCloud<pcl::PointXYZINormal>::Ptr VoxelGridFilterNorm(pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud)
{
    //замена точек в вокселе одной точкой 
    // Step 2: Downsample using Voxel Grid Filter
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::VoxelGrid<pcl::PointXYZINormal> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize((float)LeafSize2_int / 100, (float)LeafSize2_int / 100, (float)LeafSize2_int / 100);  // Adjust the leaf size as needed //0.1
    voxel_filter.filter(*output_cloud);

    return output_cloud;
}


pcl::PointCloud<pcl::PointXYZINormal>::Ptr Smoothing(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{   
    //сглаживание всех точек
    // Step 3: Smooth using Moving Least Squares
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZINormal> mls;  // Updated to output PointNormal
    mls.setInputCloud(input_cloud);
    mls.setSearchRadius((double)SearchRadius_int / 100);  // Set the smoothing radius //0.2
    mls.setPolynomialOrder(PolynomialOrder_int);  //2
    mls.setComputeNormals(true);  // Enable normal computation (needed for meshing)
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    mls.setSearchMethod(tree);
    mls.process(*output_cloud);

    return output_cloud;
}


pcl::PolygonMesh Meshing(pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud)
{
    //Создание сетки
    // Step 4: Create a mesh using Greedy Projection Triangulation
    pcl::GreedyProjectionTriangulation<pcl::PointXYZINormal> gp3;
    pcl::PolygonMesh output_mesh;
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree2->setInputCloud(input_cloud);
    // Set the parameters
    gp3.setSearchRadius((double)MeshSearchRadius_int / 100);  // Set the maximum distance between connected points (adjust as needed) 0.5
    gp3.setMu((double)Mu_int / 10);  // Multiplicative factor for search radius 2.5
    gp3.setMaximumNearestNeighbors(MaximumNearestNeighbors_int);  // Limit the number of nearest neighbors //100
    
    gp3.setMaximumSurfaceAngle(M_PI / ((double)MaximumSurfaceAngle_int / 10));  // Maximum angle (in radians) between surface normals // pi/4
    gp3.setMinimumAngle(M_PI / (double)MinimumAngle_int);  // Minimum angle for a triangle // pi/18
    gp3.setMaximumAngle(2 * M_PI / ((double)MaximumAngle_int / 10));  // Maximum angle for a triangle // 2*pi/3
    gp3.setNormalConsistency(false);

    // Generate the mesh
    gp3.setInputCloud(input_cloud);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(output_mesh);

    return output_mesh;

}


int main() {  
    create_trackbar();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    cloud = Load_cloud_from_file();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intens(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PolygonMesh mesh; 

    // Создайте визуализатор и настройте его
    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.setBackgroundColor(0.2, 0.2, 0.2); // Черный фон
    viewer.addCoordinateSystem(0.2); // Добавить систему координат
    viewer.initCameraParameters();

    // Цикл визуализации
    while (!viewer.wasStopped()) { //q     
        if (treckbar_flag == 1) 
        {
            treckbar_flag = 0;
            int flag_idx = 0;
            viewer.removePointCloud("cloud");
            viewer.removePolygonMesh("mesh");

            *cloud_intens = *cloud;
            if (points_flag == flag_idx){ //0
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_intens, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(cloud_intens, intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                std::cout << "cloud" << std::endl;
            }
            flag_idx++;            


            cloud_intens = PassThroughFilter(cloud_intens);
            if (points_flag == flag_idx) //1
            {
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_intens, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(cloud_intens, intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                std::cout << "PassThroughFilter" << std::endl;
            }
            flag_idx++;          


            cloud_intens = SorFilter(cloud_intens);
            if (points_flag == flag_idx) //2
            {
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_intens, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(cloud_intens, intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                std::cout << "SorFilter" << std::endl;
            }
            flag_idx++;
            

            cloud_intens = VoxelGridFilter(cloud_intens);
            if (points_flag == flag_idx) //3
            {
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_intens, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(cloud_intens, intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                std::cout << "VoxelGridFilter" << std::endl;
            }
            flag_idx++;


            cloud_intens = SmoothByIntensity(cloud_intens);
            if (points_flag == flag_idx) //4
            {
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_intens, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(cloud_intens, intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                std::cout << "SmoothByIntensity" << std::endl;
            }
            flag_idx++;


            cloud_normals = Smoothing(cloud_intens);
            if (points_flag == flag_idx) //5
            {
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> normal_intensity_distribution(cloud_normals, "intensity");
                viewer.addPointCloud<pcl::PointXYZINormal>(cloud_normals, normal_intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                std::cout << "Smoothing" << std::endl;
            }
            flag_idx++;


            //cloud_normals = VoxelGridFilterNorm(cloud_normals);
            if (points_flag == flag_idx) //6
            {
                cloud_normals = VoxelGridFilterNorm(cloud_normals);
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> normal_intensity_distribution(cloud_normals, "intensity");
                viewer.addPointCloud<pcl::PointXYZINormal>(cloud_normals, normal_intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                std::cout << "VoxelGridFilterNorm" << std::endl;
            }
            flag_idx++;


            mesh = Meshing(cloud_normals);
            if (mesh_flag == 1)
                viewer.addPolygonMesh(mesh, "mesh");
        }
        int key = cv::waitKey(1); 
        viewer.spinOnce(100); 
    }

    return 0;
}