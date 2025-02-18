#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/PolygonMesh.h>

int main()
{
    // Загрузка облака точек
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("cloud.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file cloud.pcd \n");
        return (-1);
    }

    // Создание объекта Marching Cubes
    pcl::MarchingCubesHoppe<pcl::PointXYZ> mc;
    mc.setInputCloud(cloud);  // Задание облака точек
    mc.setGridResolution(50, 50, 50);  // Задание разрешения сетки

    // Реконструкция поверхности
    pcl::PolygonMesh mesh;
    mc.reconstruct(mesh);


    return 0;
}
