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
#include <pcl/surface/poisson.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <unordered_set>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/features/normal_3d.h>

#include <opencv2/opencv.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
/* 10 35 2 50 50 100 12 72 12 */
/*10 100 7 70 6 12 2 50 50 100 12 72 12 3 0*/
/*10 100 8 70 4 12 2 53 59 50 10 4 10 10 5*/
/*4 1 10 100 0 70 9 41 2 50 30 100 12 72 12*/

int search_radius_int = 2;

int low_intens_int = 10;
int high_intens_int = 100;

int number_of_neighbors_int = 0; 
int distance_to_neighbors_int = 70; 

int LeafSize_int = 10; 

int SearchRadius_int = 18;
int PolynomialOrder_int = 1;

int Smoothing_for_int = 1; 

int MeshSearchRadius_int = 50;
int Mu_int = 100;
int MaximumNearestNeighbors_int = 100;

int MaximumSurfaceAngle_int = 100;
int MinimumAngle_int = 2;
int MaximumAngle_int = 100;

int Poasson_Depth_int = 2;
int Poasson_Scale_int = 10;

int SolverDivide_int = 5;
int IsoDivide_int = 5;
int SamplesPerNode_int = 12;

int outputPolygons_int = 1;

int PoassonClaster_Depth_int = 3;
int PoassonClaster_Scale_int = 10;
int PoassonClaster_Tolerance_int = 10;
int MinClusterSize_int = 10;
int MaxClusterSize_int = 2000;

int MarchingCubes_KSearch_int = 20;
int MarchingCubes_IsoLevel_int = 1;
int MarchingCubes_GridResolution_int = 50;


bool treckbar_flag = 1;

int points_flag = 0;
int mesh_flag = 0;

void on_trackbar(int, void*) {
    treckbar_flag = 1;
}

void create_trackbar()
{
    // Создайте окно для трекбаров
    cv::namedWindow("Controls", cv::WINDOW_NORMAL);
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
            std::cout << point.x << " " << point.y << " " << point.z << std::endl;
        }
    }
    inputFile.close();

    return cloud;
}




pcl::PointCloud<pcl::PointXYZI>::Ptr SmoothByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, int number) 
{ //сглаживающий фильтр учитывающий интенсивности
    cv::createTrackbar(std::to_string(number) + ") SmoothByIntensity: search_radius", "Controls", &search_radius_int, 6, on_trackbar);
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


pcl::PointCloud<pcl::PointXYZI>::Ptr PassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, int number)
{
    cv::createTrackbar(std::to_string(number) + ") PassThroughFilter: low_intens", "Controls", &low_intens_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") PassThroughFilter: high_intens", "Controls", &high_intens_int, 100, on_trackbar);
    //фильтр по интенсивностям
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass_filter;
    pass_filter.setInputCloud(input_cloud);
    pass_filter.setFilterFieldName("intensity"); // Фильтрация по полю интенсивности
    pass_filter.setFilterLimits((float)low_intens_int, (float)high_intens_int); // Удаление точек с интенсивностью менее 0.2 и более 1.0
    pass_filter.filter(*output_cloud);

    return output_cloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr SorFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, int number)
{ //фильтр удаляющий точки в зависимости от количества соседей на расстоянии от точки
    cv::createTrackbar(std::to_string(number) + ") SorFilter: number_of_neighbors", "Controls", &number_of_neighbors_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") SorFilter: distance_to_neighbors", "Controls", &distance_to_neighbors_int, 100, on_trackbar);

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;
    sor_filter.setInputCloud(input_cloud);
    sor_filter.setMeanK(number_of_neighbors_int); // Количество соседей для анализа (например, 50)//20
    sor_filter.setStddevMulThresh((double)distance_to_neighbors_int / 100); // Удаляет точки, которые находятся дальше чем 1 стандартное отклонение от среднего расстояния //1.4
    sor_filter.filter(*output_cloud);

    return output_cloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr VoxelGridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, int number)
{
    cv::createTrackbar(std::to_string(number) + ") VoxelGridFilter: LeafSize", "Controls", &LeafSize_int, 100, on_trackbar);
    //замена точек в вокселе одной точкой 
    // Step 2: Downsample using Voxel Grid Filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize((float)LeafSize_int /** 3*/ / 100, (float)LeafSize_int /** 3*/  / 100, (float)LeafSize_int / 100);  // Adjust the leaf size as needed //0.1
    voxel_filter.filter(*output_cloud);

    return output_cloud;
}


pcl::PointCloud<pcl::PointXYZINormal>::Ptr Smoothing(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, int number)
{   
    cv::createTrackbar(std::to_string(number) + ") Smoothing: SearchRadius", "Controls", &SearchRadius_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") Smoothing: PolynomialOrder", "Controls", &PolynomialOrder_int, 5, on_trackbar);
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


pcl::PolygonMesh Meshing(pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud, int number)
{
    cv::createTrackbar(std::to_string(number) + ") Meshing: MeshSearchRadius", "Controls", &MeshSearchRadius_int, 200, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") Meshing: Mu", "Controls", &Mu_int, 500, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") Meshing: MaximumNearestNeighbors", "Controls", &MaximumNearestNeighbors_int, 500, on_trackbar);

    cv::createTrackbar(std::to_string(number) + ") Meshing: MaximumSurfaceAngle", "Controls", &MaximumSurfaceAngle_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") Meshing: MinimumAngle", "Controls", &MinimumAngle_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") Meshing: MaximumAngle", "Controls", &MaximumAngle_int, 100, on_trackbar);
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
    
    gp3.setMaximumSurfaceAngle(M_PI / 100 * (double)MaximumSurfaceAngle_int);  // Maximum angle (in radians) between surface normals // pi/4
    gp3.setMinimumAngle(M_PI / 100 * (double)MinimumAngle_int);  // Minimum angle for a triangle // pi/18
    gp3.setMaximumAngle(2 * M_PI / 100 * (double)MaximumAngle_int);  // Maximum angle for a triangle // 2*pi/3
    gp3.setNormalConsistency(true);
    

    // Generate the mesh
    gp3.setInputCloud(input_cloud);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(output_mesh);

    return output_mesh;

}


pcl::PolygonMesh MeshingPoasson(pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud, int number)
{
    cv::createTrackbar(std::to_string(number) + ") MeshingPoasson: Depth", "Controls", &Poasson_Depth_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") MeshingPoasson: Scale", "Controls", &Poasson_Scale_int, 100, on_trackbar);

    cv::createTrackbar(std::to_string(number) + ") MeshingPoasson: IsoDivide", "Controls", &IsoDivide_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") MeshingPoasson: outputPolygons", "Controls", &outputPolygons_int, 1, on_trackbar);

    pcl::Poisson<pcl::PointXYZINormal> poisson;
    poisson.setInputCloud(input_cloud);
    poisson.setDepth(Poasson_Depth_int);  // Отрегулируйте в зависимости от ваших данных
    //poisson.setScale((float)Poasson_Scale_int / 10);  // Отрегулируйте для гладкости

    //poisson.setIsoDivide(IsoDivide_int);
    //poisson.setOutputPolygons((bool)outputPolygons_int);

    pcl::PolygonMesh output_mesh;
    poisson.reconstruct(output_mesh);

    return output_mesh;

}


void MeshingPoassonClaster2(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, pcl::visualization::PCLVisualizer* viewer, int number)
{
    cv::createTrackbar(std::to_string(number) + ") MeshingPoassonClaster: Tolerance", "Controls", &PoassonClaster_Tolerance_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") MeshingPoassonClaster: MinClusterSize", "Controls", &MinClusterSize_int, 500, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") MeshingPoassonClaster: MaxClusterSize", "Controls", &MaxClusterSize_int, 5000, on_trackbar);

    cv::createTrackbar(std::to_string(number) + ") MeshingPoassonClaster: Depth", "Controls", &PoassonClaster_Depth_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") MeshingPoassonClaster: Scale", "Controls", &PoassonClaster_Scale_int, 100, on_trackbar);

    cv::createTrackbar(std::to_string(number) + ") MeshingPoassonClaster: SolverDivide", "Controls", &SolverDivide_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") MeshingPoassonClaster: IsoDivide", "Controls", &IsoDivide_int, 100, on_trackbar);
    cv::createTrackbar(std::to_string(number) + ") MeshingPoassonClaster: SamplesPerNode", "Controls", &SamplesPerNode_int, 100, on_trackbar);

    // Настройка поиска соседей
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree->setInputCloud(cloud);

    // Кластеризация
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
    ec.setClusterTolerance((double)PoassonClaster_Tolerance_int / 100); // расстояние между точками в одном кластере
    ec.setMinClusterSize(MinClusterSize_int);    // минимальное количество точек в кластере
    ec.setMaxClusterSize(MaxClusterSize_int);  // максимальное количество точек в кластере
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> clusters;

    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZINormal>);

        for (const auto& index : indices.indices) {
            cluster->points.push_back(cloud->points[index]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }


    std::vector<pcl::PolygonMesh> meshes;

    for (const auto& cluster : clusters) {
        pcl::Poisson<pcl::PointXYZINormal> poisson;
        poisson.setDepth(PoassonClaster_Depth_int); // параметры, которые можно варьировать
        poisson.setScale((double)PoassonClaster_Scale_int / 10);
        poisson.setSolverDivide(SolverDivide_int);  // Параметры сетки, управляющие детализацией//8
        poisson.setIsoDivide(IsoDivide_int); //8
        poisson.setSamplesPerNode((float)SamplesPerNode_int / 10);  // Количество точек на узел //1.5
        poisson.setInputCloud(cluster);

        pcl::PolygonMesh mesh;
        poisson.reconstruct(mesh);
        meshes.push_back(mesh);
    }

    int mesh_id = 0;
    for (const auto& mesh : meshes) {
        // Генерируем уникальное имя для каждой сетки, чтобы добавлять их по отдельности
        std::string mesh_name = "mesh_" + std::to_string(mesh_id++);
        
        // Добавляем сетку в визуализатор
        viewer->removePolygonMesh(mesh_name);
        viewer->addPolygonMesh(mesh, mesh_name);
        
        // Настраиваем цвет для сетки, чтобы различать их
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, mesh_name); // например, зелёный цвет
    }
}

void intensToRgb(float h, float &r, float &g, float &b) {
    // Величина цвета
    float x = 1.0 * (1 - std::fabs(fmod(h / 60.0, 2) - 1)); // Промежуточное значение

    float r_, g_, b_;
    if (h >= 0 && h < 60) {
        r_ = 1.0; g_ = x; b_ = 0;
    } else if (h >= 60 && h < 120) {
        r_ = x; g_ = 1.0; b_ = 0;
    } else if (h >= 120 && h < 180) {
        r_ = 0; g_ = 1.0; b_ = x;
    } else if (h >= 180 && h < 240) {
        r_ = 0; g_ = x; b_ = 1.0;
    } else if (h >= 240 && h < 300) {
        r_ = x; g_ = 0; b_ = 1.0;
    } else {
        r_ = 1.0; g_ = 0; b_ = x;
    }

    r = r_ + 0.0;
    g = g_ + 0.0;
    b = b_ + 0.0;
}


void ViewVoxels(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::visualization::PCLVisualizer* viewer, float leaf_size = 0.1)
{
    // Уникальные координаты вокселей для отображения
    std::unordered_set<std::string> voxel_centers;
    int voxel_id = 0;

    // Проходим по всем точкам вокселизированного облака
    for (const auto& point : *cloud) {
        // Рассчитываем координаты центра каждого вокселя
        float center_x = std::round(point.x / leaf_size) * leaf_size;
        float center_y = std::round(point.y / leaf_size) * leaf_size;
        float center_z = std::round(point.z / leaf_size) * leaf_size;

        // Используем строку, чтобы определить уникальный воксель
        std::string voxel_key = std::to_string(center_x) + "_" + std::to_string(center_y) + "_" + std::to_string(center_z);
        if (voxel_centers.find(voxel_key) != voxel_centers.end()) {
            continue;
        }
        voxel_centers.insert(voxel_key);

        // Добавляем куб для каждого вокселя в визуализатор
        viewer->addCube(
            center_x - leaf_size / 2, center_x + leaf_size / 2,
            center_y - leaf_size / 2, center_y + leaf_size / 2,
            center_z - leaf_size / 2, center_z + leaf_size / 2,
            1.0, 1.0, 1.0, // Цвет куба
            "voxel_" + std::to_string(voxel_id++)
        );

        float normalized_intensity = point.intensity * 3;
        float r, g, b;
        intensToRgb(normalized_intensity, r, g, b);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "voxel_" + std::to_string(voxel_id - 1));
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 1.0, "voxel_" + std::to_string(voxel_id - 1));
    }
}


void view_cloud (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::visualization::PCLVisualizer* viewer)
{
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
}

void view_normslize_cloud (pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, pcl::visualization::PCLVisualizer* viewer)
{
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> normal_intensity_distribution(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZINormal>(cloud, normal_intensity_distribution, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
}


void convertPointCloudXYZIToXYZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) {
    // Очистить выходное облако на случай, если оно уже содержит данные
    cloud_out->clear();

    // Задать размерность и параметры выходного облака
    cloud_out->width = cloud_in->width;
    cloud_out->height = cloud_in->height;
    cloud_out->is_dense = cloud_in->is_dense;
    cloud_out->points.resize(cloud_in->points.size());

    // Копировать только координаты x, y, z из входного облака
    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
        cloud_out->points[i].x = cloud_in->points[i].x;
        cloud_out->points[i].y = cloud_in->points[i].y;
        cloud_out->points[i].z = cloud_in->points[i].z;
    }
}


pcl::PolygonMesh MarchingCubes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cv::createTrackbar("MarchingCubes: KSearch", "Controls", &MarchingCubes_KSearch_int, 100, on_trackbar);
    cv::createTrackbar("MarchingCubes: IsoLevel", "Controls", &MarchingCubes_IsoLevel_int, 100, on_trackbar);
    cv::createTrackbar("MarchingCubes: GridResolution", "Controls", &MarchingCubes_GridResolution_int, 100, on_trackbar);

    // Вычисляем нормали для точек, так как MarchingCubesHoppe требует их наличия
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(MarchingCubes_KSearch_int); //20
    ne.compute(*normals);

    // Объединяем точки и их нормали
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Инициализируем Marching Cubes Hoppe
    pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
    mc.setInputCloud(cloud_with_normals);
    mc.setIsoLevel((float)MarchingCubes_IsoLevel_int / 100); //0.0
    mc.setGridResolution(MarchingCubes_GridResolution_int, MarchingCubes_GridResolution_int, MarchingCubes_GridResolution_int);

    pcl::PolygonMesh mesh;
    mc.reconstruct(mesh);

    return mesh;
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
            viewer.removeAllShapes();

            *cloud_intens = *cloud;
            if (points_flag == flag_idx) { //0
                view_cloud(cloud_intens, &viewer);
                std::cout << "cloud" << std::endl;
            }
            flag_idx++;            


            cloud_intens = PassThroughFilter(cloud_intens, flag_idx);
            if (points_flag == flag_idx){
                view_cloud(cloud_intens, &viewer);
                std::cout << "PassThroughFilter" << std::endl;
            }
            flag_idx++;          


            cloud_intens = SorFilter(cloud_intens, flag_idx);
            if (points_flag == flag_idx) {
                view_cloud(cloud_intens, &viewer);
                std::cout << "SorFilter" << std::endl;
            }
            flag_idx++;
            

            cloud_intens = VoxelGridFilter(cloud_intens, flag_idx);
            if (points_flag == flag_idx) {
                view_cloud(cloud_intens, &viewer);
                std::cout << "VoxelGridFilter" << std::endl;
            }
            flag_idx++;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_intens_no_N(new pcl::PointCloud<pcl::PointXYZ>());
            convertPointCloudXYZIToXYZ(cloud_intens, cloud_intens_no_N);
            if (mesh_flag == 1) {  
                mesh = MarchingCubes(cloud_intens_no_N);            
                viewer.addPolygonMesh(mesh, "mesh");
                //ViewVoxels(cloud_intens, &viewer, 0.1);
            }
        }
        int key = cv::waitKey(1); 
        viewer.spinOnce(100); 
    }

    return 0;
}