#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

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
        if (numbers.size() >= 4) {
            pcl::PointXYZ point;
            point.x = numbers[0];
            point.y = numbers[1];
            point.z = numbers[2];
            cloud->points.push_back(point);
        }
    }
    inputFile.close();

    return cloud;
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = Load_cloud_from_file();
    
    // Вычисляем нормали для точек, так как MarchingCubesHoppe требует их наличия
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*normals);

    // Объединяем точки и их нормали
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Инициализируем Marching Cubes Hoppe
    pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
    mc.setInputCloud(cloud_with_normals);
    mc.setIsoLevel(0.0);
    mc.setGridResolution(50, 50, 50);

    pcl::PolygonMesh mesh;
    mc.reconstruct(mesh);


    // Визуализация
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPolygonMesh(mesh, "mesh");
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "input_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
