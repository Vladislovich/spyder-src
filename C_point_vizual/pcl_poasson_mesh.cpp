#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>  // Включите заголовок Poisson
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
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

    // Шаг 2: Уменьшение размера с помощью фильтра Voxel Grid
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // Настройте размер листа по мере необходимости
    voxel_filter.filter(*cloud_filtered);

    // Шаг 3: Сглаживание с помощью метода наименьших квадратов
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setInputCloud(cloud_filtered);
    mls.setSearchRadius(0.2);  // Установите радиус сглаживания
    mls.setPolynomialOrder(2);  // Включите полиномиальную аппроксимацию
    mls.setComputeNormals(true);  // Включите вычисление нормалей (необходимо для создания сетки)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    mls.setSearchMethod(tree);
    mls.process(*cloud_smoothed_with_normals);

    // Шаг 4: Создание сетки с использованием реконструкции поверхности Поассона
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setInputCloud(cloud_smoothed_with_normals);
    poisson.setDepth(12);  // Отрегулируйте в зависимости от ваших данных
    poisson.setScale(1.1);  // Отрегулируйте для гладкости
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    // Сохранение сетки в файл
    pcl::io::savePLYFile("../points_and_meshes/smoothed_mesh.ply", mesh);
    std::cout << "Сетка сохранена как 'smoothed_mesh.ply'" << std::endl;

    return 0;
}