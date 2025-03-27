#include "PCL.hpp"

int main()
{
    std::string filename;
    std::cout << "Введите файл: " << std::endl;
    std::getline(std::cin, filename);    
    mesh_struct Mesh("../show/" + filename);

    static int point_viz = 0;
    global_treckbar.push_treckbar("point_viz", &point_viz, 5);    
    static int mesh_viz = 0;
    global_treckbar.push_treckbar("mesh_viz", &mesh_viz, 1);

    //Создание визуализатора
    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.setBackgroundColor(1.0, 1.0, 1.0); 
    viewer.addCoordinateSystem(0.2); // Добавить систему координат
    viewer.setCameraPosition(0, 1, 1, 1, 1, 1);
    
    global_treckbar.treckbar_flag = true;
    
    while (!viewer.wasStopped()) 
    {
        if (global_treckbar.treckbar_flag)
        {
            Mesh.Load_cloud_from_file("../show/" + filename);
            global_treckbar.treckbar_flag = 0;

            viewer.removePointCloud("cloud");
            viewer.removePolygonMesh("mesh");

            Mesh.cloudXYZI = Mesh.cloudXYZI_src;
            if (point_viz == 0) Mesh.cloudXYZI_viz = Mesh.cloudXYZI; 
            Mesh.cloudXYZI = Mesh.VoxelGridFilter(Mesh.cloudXYZI);
            if (point_viz == 1) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;
            Mesh.cloudXYZIN = Mesh.Smoothing(Mesh.cloudXYZI);
            if (point_viz == 2) Mesh.cloudXYZI_viz = Mesh.convertToXYZI(Mesh.cloudXYZIN);
            if (point_viz <= 2){
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(Mesh.cloudXYZI_viz, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(Mesh.cloudXYZI_viz, intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
            }
            if (mesh_viz == 1){
                Mesh.mesh = Mesh.Poisson_mesh(Mesh.cloudXYZIN);             
                viewer.addPolygonMesh(*Mesh.mesh, "mesh");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "mesh");
            }                        
        }        
        cv::waitKey(1); 
        viewer.spinOnce(100); 
    }         
}