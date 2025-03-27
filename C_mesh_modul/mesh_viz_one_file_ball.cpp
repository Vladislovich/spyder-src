#include "PCL.hpp"

int main()
{
    std::string filename;
    std::cout << "Введите файл: ";
    std::getline(std::cin, filename);    
    mesh_struct Mesh("../show/" + filename);

    static int point_viz = 0;
    global_treckbar.push_treckbar("point_viz", &point_viz, 5);    

    static int ref_mesh_viz = 0;
    global_treckbar.push_treckbar("ref_mesh_viz", &ref_mesh_viz, 1);
    static int count_normal_error = 0;
    global_treckbar.push_treckbar("count_normal_error", &count_normal_error, 1);


    //Создание визуализатора
    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.setBackgroundColor(1.0, 1.0, 1.0); // Черный фон
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
            viewer.removeShape("ransac_sphere");

            Mesh.cloudXYZI = Mesh.cloudXYZI_src;
            if (point_viz == 0) Mesh.cloudXYZI_viz = Mesh.cloudXYZI; 

            Mesh.cloudXYZI = Mesh.PassThroughFilter(Mesh.cloudXYZI_src);
            if (point_viz == 1) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

            Mesh.cloudXYZI = Mesh.VoxelGridFilter(Mesh.cloudXYZI);
            if (point_viz == 2) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;
            Mesh.RefXYZI = Mesh.cloudXYZI;


            if (point_viz <= 2){
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(Mesh.cloudXYZI_viz, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(Mesh.cloudXYZI_viz, intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
            }

            std::vector<float> coef = Mesh.FitFixedRadiusSphere(Mesh.RefXYZI, 0.3);    
            if (coef.empty()) continue; 

            if (ref_mesh_viz == 1)
                viewer.addSphere(pcl::PointXYZ(coef[0], coef[1], coef[2]), coef[3], 0.0, 0.0, 1.0, "ransac_sphere");
                
            if (count_normal_error)
                std::cout << "normal error: " << Mesh.count_normal_error_ball(coef, Mesh.RefXYZI) << std::endl; 
        }        
        cv::waitKey(1); 
        viewer.spinOnce(100); 
    }         
}