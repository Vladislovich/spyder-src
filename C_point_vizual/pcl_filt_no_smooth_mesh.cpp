#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
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

    // Шаг 1: Загрузка точек из файла
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

    // Шаг 2: Уменьшение размера с использованием фильтра Voxel Grid
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);  // Настройте размер листа по мере необходимости
    voxel_filter.filter(*cloud_filtered);

    // Сохранение уменьшенного облака точек
    pcl::io::savePCDFileASCII("../points_and_meshes/filtered_points.pcd", *cloud_filtered);
    std::cout << "Downsampled point cloud saved as 'filtered_points.pcd'" << std::endl;

    // Шаг 3: Создание сетки с использованием Greedy Projection Triangulation
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud_filtered);
    ne.setInputCloud(cloud_filtered);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*normals);

    pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);

     // Save the smoothed point cloud
    pcl::io::savePCDFileASCII("../points_and_meshes/normals_points.pcd", *cloud_with_normals);
    std::cout << "Downsampled and smoothed point cloud saved as 'normals_points.pcd'" << std::endl;

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Настройка параметров
    gp3.setSearchRadius(0.25);  // Максимальное расстояние между соединенными точками (настраивайте при необходимости)
    gp3.setMu(2.5);  // Множитель радиуса поиска
    gp3.setMaximumNearestNeighbors(100);  // Лимит на количество ближайших соседей
    gp3.setMaximumSurfaceAngle(M_PI / 4);  // Максимальный угол (в радианах) между нормалями поверхности
    gp3.setMinimumAngle(M_PI / 18);  // Минимальный угол для треугольника
    gp3.setMaximumAngle(2 * M_PI / 3);  // Максимальный угол для треугольника
    gp3.setNormalConsistency(false);

    // Генерация сетки
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(mesh);

    // Сохранение сетки в файл
    pcl::io::savePLYFile("../points_and_meshes/mesh_no_smooth.ply", mesh);
    std::cout << "Mesh saved as 'mesh_no_smooth.ply'" << std::endl;

    return 0;
}