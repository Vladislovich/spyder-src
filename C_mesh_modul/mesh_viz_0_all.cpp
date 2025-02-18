#include "PCL.hpp"

int main()
{
    static int point_viz = 0;
    global_treckbar.push_treckbar("point_viz", &point_viz, 10);
    static int mesh_viz = 0;
    global_treckbar.push_treckbar("mesh_viz", &mesh_viz, 2);

    //mesh_struct Mesh("../data/lidar_dataset_2.txt", 2);
    mesh_struct Mesh("../data/velodyne_data_vert_07_5.txt");
    
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

            if (point_viz == 0) Mesh.cloudXYZI_viz = Mesh.cloudXYZI_src;

            Mesh.cloudXYZI = Mesh.PassThroughFilter(Mesh.cloudXYZI_src);
            if (point_viz == 1) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

            Mesh.cloudXYZI = Mesh.XYZSelection(Mesh.cloudXYZI);
            if (point_viz == 2) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

            Mesh.cloudXYZI = Mesh.VoxelGridFilter(Mesh.cloudXYZI);
            if (point_viz == 3) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

            //Mesh.cloudXYZIN = Mesh.computeNormals(Mesh.cloudXYZI);
            Mesh.cloudXYZIN = Mesh.Smoothing(Mesh.cloudXYZI);
            if (point_viz == 4) Mesh.cloudXYZI_viz = Mesh.convertToXYZI(Mesh.cloudXYZIN);

            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(Mesh.cloudXYZI_viz, "intensity");
            viewer.addPointCloud<pcl::PointXYZI>(Mesh.cloudXYZI_viz, intensity_distribution, "cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

            
            if (mesh_viz == 1)
            {
                //Mesh.mesh = Mesh.Meshing(Mesh.cloudXYZIN);
                Mesh.mesh = Mesh.Poisson_mesh(Mesh.cloudXYZIN);
                //Mesh.mesh = Mesh.FastMesh(Mesh.cloudXYZIN);  
                //Mesh.mesh = Mesh.Alpha_shapes(Mesh.cloudXYZIN);        
                //Mesh.mesh = Mesh.MarchingCubes(Mesh.cloudXYZIN);              
                viewer.addPolygonMesh(*Mesh.mesh, "mesh");
            }
        }
        int key = cv::waitKey(1); 
        viewer.spinOnce(100); 
    }
}