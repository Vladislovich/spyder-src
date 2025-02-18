#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>

int main() {
    std::string input_file = "pointcloud_log_3.txt";
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
        cloud->points.push_back(pcl::PointXYZ(numbers[0], numbers[1], numbers[2]));
    }

    cloud->width = 100;//cloud->points.size();
    cloud->height = 16;
    cloud->is_dense = true;

    std::cout << "Loaded " << cloud->points.size() << " data points from " << input_file << std::endl;

    // Шаг 2: Обработка облака точек (упрощение и сглаживание)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    // Сглаживание с использованием метода Moving Least Squares (MLS)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(cloud_filtered);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);
    mls.process(*cloud_smoothed);

    // Шаг 3: Создание KdTree для поиска ближайших соседей
    pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal>);

    // Шаг 4: Триангуляция с использованием Greedy Projection Triangulation
    pcl::PolygonMesh triangles;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // Расчет нормалей
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_smoothed);
    ne.setKSearch(20);
    ne.compute(*normals);

    // Объединение координат точек и нормалей
    pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);

    kdtree->setInputCloud(cloud_with_normals);

    gp3.setSearchRadius(0.025);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 градусов
    gp3.setMinimumAngle(M_PI / 18);       // 10 градусов
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 градусов
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(kdtree);
    gp3.reconstruct(triangles);

    // Шаг 5: Сохранение триангулированной сетки в файл
    pcl::io::savePLYFile("output_mesh.ply", triangles);

    std::cout << "Mesh saved to output_mesh.ply" << std::endl;

    return 0;
}