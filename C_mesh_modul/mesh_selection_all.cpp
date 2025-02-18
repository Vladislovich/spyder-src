#include "PCL.hpp"
//std::vector<pcl::visualization::Camera> cam;


int main()
{
    static int point_viz = 0;
    global_treckbar.push_treckbar("point_viz", &point_viz, 10);
    static int mesh_viz = 0;
    global_treckbar.push_treckbar("mesh_viz", &mesh_viz, 2);


    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.setBackgroundColor(0.2, 0.2, 0.2); // Черный фон
    viewer.addCoordinateSystem(0.2); // Добавить систему координат
    viewer.setCameraPosition(1.9715, -0.752592, 1.95634, 0.381396, -0.517583, 0.765927, 0.497065, 0.0119868, 0.00990051);

    std::string path = "../exper_data";
    for (const auto& entry : fs::directory_iterator(path))
    {
        std::cout << entry.path().filename();
        mesh_struct Mesh = mesh_struct(entry.path());
        global_treckbar.treckbar_flag = true;
        int key = 0;
        //Создание визуализатора
        while (key != 'q') 
        {
            if (global_treckbar.treckbar_flag)
            {
                global_treckbar.treckbar_flag = 0;
                viewer.removePointCloud("cloud");
                viewer.removePolygonMesh("mesh");

                if (point_viz == 0) Mesh.cloudXYZI_viz = Mesh.cloudXYZI_src;

                Mesh.cloudXYZI = Mesh.PassThroughFilter(Mesh.cloudXYZI_src);
                if (point_viz == 1) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

                Mesh.cloudXYZI = Mesh.XYZSelection(Mesh.cloudXYZI_src);
                if (point_viz == 2) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(Mesh.cloudXYZI_viz, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(Mesh.cloudXYZI_viz, intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                
                if (mesh_viz == 1)
                    Mesh.Write_cloud_to_file("../exper_data_filtered/" + entry.path().filename().string());
            }
            key = cv::waitKey(1); 
            viewer.spinOnce(100); 
        }
        /*viewer.getCameras(cam);
        cout << "Cam: " << endl 
             << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl 
             << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl 
             << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;*/
    }
}