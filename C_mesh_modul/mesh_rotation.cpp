#include "PCL.hpp"

int main()
{
    static int point_viz = 0;
    global_treckbar.push_treckbar("point_viz", &point_viz, 10);
    static int mesh_viz = 0;
    global_treckbar.push_treckbar("mesh_viz", &mesh_viz, 2);

    std::string filename;
    std::cout << "Введите файл: " << std::endl;
    std::getline(std::cin, filename);
    
    mesh_struct Mesh("../selected_data/" + filename);
    Mesh.Load_cloud_from_file_currection("../selected_data/" + filename);
    //Создание визуализатора
    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.setBackgroundColor(0.2, 0.2, 0.2); // Черный фон
    viewer.addCoordinateSystem(0.2); // Добавить систему координат
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) 
    {
        if (global_treckbar.treckbar_flag)
        {
            global_treckbar.treckbar_flag = 0;
            viewer.removePointCloud("cloud");
            viewer.removePolygonMesh("mesh");

            Mesh.Load_cloud_from_file_currection("../selected_data/" + filename);

            if (point_viz == 0) Mesh.cloudXYZI_viz = Mesh.cloudXYZI_src;

            //Mesh.cloudXYZI = Mesh.PassThroughFilter(Mesh.cloudXYZI_src);
            //if (point_viz == 1) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

            Mesh.cloudXYZI = Mesh.XYZSelection(Mesh.cloudXYZI_src);
            if (point_viz == 2) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(Mesh.cloudXYZI_viz, "intensity");
            viewer.addPointCloud<pcl::PointXYZI>(Mesh.cloudXYZI_viz, intensity_distribution, "cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
            
            if (mesh_viz == 1)
                Mesh.Write_cloud_to_file("../selected_data/"+filename);
        }
        int key = cv::waitKey(1); 
        viewer.spinOnce(100); 
    }
}