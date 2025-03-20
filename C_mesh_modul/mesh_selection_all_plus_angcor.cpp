#include "PCL.hpp"
//std::vector<pcl::visualization::Camera> cam;


int main()
{
    static int point_viz = 0;
    global_treckbar.push_treckbar("point_viz", &point_viz, 10);
    static int write_points = 0;
    global_treckbar.push_treckbar("write_points", &write_points, 2);

    static int ref_mesh_viz = 0;
    global_treckbar.push_treckbar("ref_mesh_viz", &ref_mesh_viz, 1);
    static int count_normal_error = 0;
    global_treckbar.push_treckbar("count_normal_error", &count_normal_error, 1);


    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.setBackgroundColor(0.2, 0.2, 0.2); // Черный фон
    viewer.addCoordinateSystem(0.2); // Добавить систему координат
    viewer.setCameraPosition(1.9715, -0.752592, 1.95634, 0.381396, -0.517583, 0.765927, 0.497065, 0.0119868, 0.00990051);

    std::string path = "../ball_data";
    for (const auto& entry : fs::directory_iterator(path))
    {
        std::cout << entry.path().filename() << std::endl;
        mesh_struct Mesh = mesh_struct(entry.path());
        global_treckbar.treckbar_flag = true;
        int key = 0;
        //Создание визуализатора
        while (key != 'q') 
        {
            if (global_treckbar.treckbar_flag)
            {

                Mesh.Load_cloud_from_file(entry.path());
                global_treckbar.treckbar_flag = 0;
                viewer.removePointCloud("cloud");
                viewer.removePolygonMesh("mesh");
                viewer.removeShape("ransac_sphere");


                if (point_viz == 0) Mesh.cloudXYZI_viz = Mesh.cloudXYZI_src;

                Mesh.cloudXYZI = Mesh.PassThroughFilter(Mesh.cloudXYZI_src);
                if (point_viz == 1) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

                Mesh.cloudXYZI = Mesh.XYZSelection(Mesh.cloudXYZI);
                if (point_viz == 2) Mesh.cloudXYZI_viz = Mesh.cloudXYZI;

                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(Mesh.cloudXYZI_viz, "intensity");
                viewer.addPointCloud<pcl::PointXYZI>(Mesh.cloudXYZI_viz, intensity_distribution, "cloud");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                
                if (write_points == 1)
                    Mesh.Write_cloud_to_file("../ball_data_filtered/" + entry.path().filename().string());
                    //Mesh.Write_cloud_to_file("../exper_data_filtered/" + entry.path().filename().string());



                Mesh.RefXYZI = Mesh.cloudXYZI;      
                std::vector<float> coef = Mesh.RansacBall(Mesh.RefXYZI);
                coef[3] = 0.6;                   
                if (ref_mesh_viz == 1)
                {
                    double x = coef[0];
                    double y = coef[1];
                    double z = coef[2];
                    double radius = coef[3];
                
                    viewer.addSphere(pcl::PointXYZ(x, y, z), radius, 1.0, 0.0, 0.0, "ransac_sphere"); // Красная сфера
                }
                if (count_normal_error)
                    std::cout << "normal error: " << Mesh.count_normal_error_ball(coef, Mesh.cloudXYZI) << std::endl;
            }
            key = cv::waitKey(1); 
            viewer.spinOnce(100); 
        }
    }
}