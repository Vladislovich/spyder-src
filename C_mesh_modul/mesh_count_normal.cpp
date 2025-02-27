#include "PCL.hpp"

int main()
{
    static int point_viz = 0;
    global_treckbar.push_treckbar("point_viz", &point_viz, 5);    
    static int mesh_viz = 0;
    global_treckbar.push_treckbar("mesh_viz", &mesh_viz, 1);

    static int ref_point_viz = 0;
    global_treckbar.push_treckbar("ref_point_viz", &ref_point_viz, 5);
    static int ref_mesh_viz = 0;
    global_treckbar.push_treckbar("ref_mesh_viz", &ref_mesh_viz, 1);

    static int count_normal_error = 0;
    global_treckbar.push_treckbar("count_normal_error", &count_normal_error, 1);

    std::string filename;
    std::cout << "Введите файл: " << std::endl;
    std::getline(std::cin, filename);
    
    mesh_struct Mesh("../exper_data_filtered/" + filename);
    
    //Создание визуализатора
    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.setBackgroundColor(0.3, 0.3, 0.3); // Черный фон
    viewer.addCoordinateSystem(0.2); // Добавить систему координат
    viewer.setCameraPosition(0, 1, 1, 1, 1, 1);
    //viewer.initCameraParameters();
    while (!viewer.wasStopped()) 
    {
        if (global_treckbar.treckbar_flag)
        {
            global_treckbar.treckbar_flag = 0;

            viewer.removePointCloud("cloud");
            viewer.removePolygonMesh("mesh");

            Mesh.cloudXYZI = Mesh.cloudXYZI_src;
            if (point_viz == 0) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;            

            Mesh.cloudXYZI = Mesh.VoxelGridFilter(Mesh.cloudXYZI);
            if (point_viz == 1) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

            Mesh.cloudXYZIN = Mesh.Smoothing(Mesh.cloudXYZI);
            if (point_viz == 2) Mesh.cloudXYZI_viz = Mesh.convertToXYZI(Mesh.cloudXYZIN);
          
            if (point_viz < 3)
            {
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(Mesh.cloudXYZI_viz, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(Mesh.cloudXYZI_viz, intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
            }
            
            if (mesh_viz == 1)
            {
                Mesh.mesh = Mesh.Poisson_mesh(Mesh.cloudXYZIN);             
                viewer.addPolygonMesh(*Mesh.mesh, "mesh");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "mesh");
            }



            viewer.removePointCloud("ref_cloud");
            viewer.removePolygonMesh("ref_mesh");
            Mesh.RefXYZI = Mesh.cloudXYZI_src;
            if (ref_point_viz == 0) Mesh.RefXYZI_viz = Mesh.RefXYZI; 

            std::vector<float> coef = Mesh.Ransac(Mesh.RefXYZI);
            Mesh.RefXYZI = Mesh.generatePlane(coef, 0.5);
            if (ref_point_viz == 1) Mesh.RefXYZI_viz = Mesh.RefXYZI;

            Mesh.RefXYZIN = Mesh.computeNormals(Mesh.RefXYZI);
            if (ref_point_viz == 2) Mesh.RefXYZI_viz = Mesh.convertToXYZI(Mesh.RefXYZIN);

            if (ref_point_viz < 3)
            {
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> ref_distribution(Mesh.RefXYZI_viz, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(Mesh.RefXYZI_viz, ref_distribution, "ref_cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ref_cloud");
            }            
            
            if (ref_mesh_viz == 1)
            {
                Mesh.ref_mesh = Mesh.Poisson_mesh(Mesh.RefXYZIN);           
                viewer.addPolygonMesh(*Mesh.ref_mesh, "ref_mesh");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "ref_mesh");
            }

            Mesh.write_point_xyz(&viewer);

            if (count_normal_error == 1)
            {
                float normal_error = Mesh.count_normal_error(coef, Mesh.cloudXYZIN);
                std::cout << "normal error: " << normal_error << std::endl;
            }
        }
        int key = cv::waitKey(1); 
        viewer.spinOnce(100); 
    }
}


// vld_n45_100_3.txt
// ord_pov45_50_2.txt