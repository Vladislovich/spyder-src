#include "PCL.hpp"

treckbar global_treckbar;

mesh_struct::mesh_struct(std::string filename)
{
    Load_cloud_from_file(filename);
}

mesh_struct::~mesh_struct()
{
}


float mesh_struct::DegreesToRadians(float degrees) 
{
    return degrees * M_PI / 180;
}


float mesh_struct::RadiansToDegrees(float rad) 
{
    return rad * 180 / M_PI;
}


void mesh_struct::Load_cloud_from_file(std::string filename)
{    
    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << "Не удалось открыть файл input.txt" << std::endl;
    }
    
    // Step 1: Load points from file
    PCLIptr cloud(new PCLI);
    std::string line;
    while (std::getline(inputFile, line)) 
    {
        std::istringstream iss(line);
        std::vector<float> numbers;
        float num;
        while (iss >> num) 
            numbers.push_back(num);
        //std::cout << "numbers.size(): " << numbers.size() << std::endl;
        if (numbers.size() == 6) 
        {
            pcl::PointXYZI point;
            //r theta ard_ang x y inten
            auto ard_ang = DegreesToRadians(numbers[2]);
            auto x = numbers[3];
            auto y = numbers[4];

            float My[3][3] = {  {cos(ard_ang),  0, sin(ard_ang) },
                                {0,             1, 0            },
                                {-sin(ard_ang), 0, cos(ard_ang) }};

            point.x = x * My[0][0] + y * My[0][1];
            point.y = x * My[1][0] + y * My[1][1];
            point.z = x * My[2][0] + y * My[2][1];
            point.intensity = numbers[5] / 2.55;

            cloud->points.push_back(point);
            //std::cout << point.x << " " << point.y << " " << point.z << std::endl; debug
        }

        if (numbers.size() == 5) {
            pcl::PointXYZI point;
            //ard_ang x y z inten
            auto ard_ang = DegreesToRadians(numbers[0]);
            auto x = numbers[1];
            auto y = numbers[2];
            auto z = numbers[3];

            float My[3][3] = {  {cos(ard_ang),  0, sin(ard_ang) },
                                {0,             1, 0            },
                                {-sin(ard_ang), 0, cos(ard_ang) }};

            point.x = x * My[0][0] + y * My[0][1] + z * My[0][2];
            point.y = x * My[1][0] + y * My[1][1] + z * My[1][2];
            point.z = x * My[2][0] + y * My[2][1] + z * My[2][2];
            point.intensity = numbers[4];

            cloud->points.push_back(point);
            //std::cout << point.x << " " << point.y << " " << point.z << std::endl; debug
        }

        if (numbers.size() == 4) {
            pcl::PointXYZI point;
            
            point.x = numbers[0];
            point.y = numbers[1];
            point.z = numbers[2];
            point.intensity = numbers[3];

            cloud->points.push_back(point);
        }
    }
    inputFile.close();
    cloudXYZI_src = cloud;
}


void mesh_struct::Load_cloud_from_file_currection(std::string filename)
{    
    static int ang_int = 1;
    global_treckbar.push_treckbar("ang_int", &ang_int, 1000);

    std::ifstream inputFile(filename);
    if (!inputFile.is_open()) {
        std::cerr << "Не удалось открыть файл input.txt" << std::endl;
    }

    float ang = -DegreesToRadians((float)ang_int / 100);

    // Step 1: Load points from file
    PCLIptr cloud(new PCLI);
    std::string line;
    while (std::getline(inputFile, line)) 
    {
        std::istringstream iss(line);
        std::vector<float> numbers;
        float num;
        while (iss >> num) 
            numbers.push_back(num);

        if (numbers.size() == 4) 
        {
            pcl::PointXYZI point;            
            
            auto x = numbers[0];
            auto y = numbers[1];
            auto z = numbers[2];

            float My[3][3] = {  {cos(ang),  0, sin(ang) },
                                {0,         1, 0        },
                                {-sin(ang), 0, cos(ang) }};

            point.x = x * My[0][0] + y * My[0][1] + z * My[0][2];
            point.y = x * My[1][0] + y * My[1][1] + z * My[1][2];
            point.z = x * My[2][0] + y * My[2][1] + z * My[2][2];
            point.intensity = numbers[3];

            cloud->points.push_back(point);
        }
    }
    inputFile.close();
    cloudXYZI_src = cloud;
}


void mesh_struct::Write_cloud_to_file(std::string filename)
{    
    std::ofstream outputFile(filename);
    if (!outputFile.is_open()) {
        std::cerr << "Не удалось открыть файл: " << filename << std::endl;
    }

    for (const auto& point : *cloudXYZI) 
    {
        float x = point.x;
        float y = point.y;
        float z = point.z;
        int intensity = point.intensity;
        outputFile << x << " " << y << " " << z << " " << intensity << std::endl;
    } 
    
    outputFile.close();
    std::cout << "Data has been written to: " << filename << std::endl;
}


PCLIptr mesh_struct::PassThroughFilter(PCLIptr input_cloud)
{    
    static int low_intens_int = 4;
    static int high_intens_int = 100;
    global_treckbar.push_treckbar("PassThroughFilter: low_intens", &low_intens_int, 100);
    global_treckbar.push_treckbar("PassThroughFilter: high_intens_int", &high_intens_int, 100);

    //фильтр по интенсивностям
    PCLIptr output_cloud(new PCLI);
    pcl::PassThrough<pcl::PointXYZI> pass_filter;
    pass_filter.setInputCloud(input_cloud);
    pass_filter.setFilterFieldName("intensity"); // Фильтрация по полю интенсивности
    pass_filter.setFilterLimits((float)low_intens_int, (float)high_intens_int); // Удаление точек с интенсивностью менее 0.2 и более 1.0
    pass_filter.filter(*output_cloud);

    return output_cloud;
}


PCLIptr mesh_struct::XYZSelection(PCLIptr input_cloud)
{    
    static int low_x_int = 1037;
    static int high_x_int = 1920;
    static int low_y_int = 967;
    static int high_y_int = 11040;
    static int low_z_int = 900;
    static int high_z_int = 2000;
    global_treckbar.push_treckbar("XYZSelection: low_x", &low_x_int, 2000);
    global_treckbar.push_treckbar("XYZSelection: high_x", &high_x_int, 2000);
    global_treckbar.push_treckbar("XYZSelection: low_y", &low_y_int, 2000);
    global_treckbar.push_treckbar("XYZSelection: high_y", &high_y_int, 2000);
    global_treckbar.push_treckbar("XYZSelection: low_z", &low_z_int, 2000);
    global_treckbar.push_treckbar("XYZSelection: high_z", &high_z_int, 2000);

    //фильтр по интенсивностям
    PCLIptr output_cloud(new PCLI);
    pcl::PassThrough<pcl::PointXYZI> pass_x_filter;
    pass_x_filter.setInputCloud(input_cloud);
    pass_x_filter.setFilterFieldName("x");
    pass_x_filter.setFilterLimits((float)(low_x_int - 1000) / 100, (float)(high_x_int - 1000) / 100);
    pass_x_filter.filter(*output_cloud);

    pcl::PassThrough<pcl::PointXYZI> pass_y_filter;
    pass_y_filter.setInputCloud(output_cloud);
    pass_y_filter.setFilterFieldName("y");
    pass_y_filter.setFilterLimits((float)(low_y_int - 1000) / 100, (float)(high_y_int - 1000) / 100);
    pass_y_filter.filter(*output_cloud);

    pcl::PassThrough<pcl::PointXYZI> pass_z_filter;
    pass_z_filter.setInputCloud(output_cloud);
    pass_z_filter.setFilterFieldName("z");
    pass_z_filter.setFilterLimits((float)(low_z_int - 1000) / 100, (float)(high_z_int - 1000) / 100);
    pass_z_filter.filter(*output_cloud);

    return output_cloud;
}




PCLIptr mesh_struct::SmoothByIntensity(PCLIptr input_cloud) 
{ //сглаживающий фильтр учитывающий интенсивности
    static int search_radius_int = 2;
    global_treckbar.push_treckbar("SmoothByIntensity: search_radius", &search_radius_int, 10);

    PCLIptr output_cloud(new PCLI);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);

    // Create a KD-tree representation of the input point cloud
    tree->setInputCloud(input_cloud);

    // Create a MovingLeastSquares object
    pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls;
    mls.setInputCloud(input_cloud);
    mls.setSearchMethod(tree);
    mls.setSearchRadius((double)search_radius_int / 10);
    mls.setPolynomialOrder(2);
    mls.setComputeNormals(false);

    // Perform the smoothing
    mls.process(*output_cloud);

    return output_cloud;
}


PCLIptr mesh_struct::SorFilter(PCLIptr input_cloud)
{ //фильтр удаляющий точки в зависимости от количества соседей на расстоянии от точки
    static int number_of_neighbors_int = 2;
    global_treckbar.push_treckbar("SorFilter: number_of_neighbors", &number_of_neighbors_int, 100);

    static int distance_to_neighbors_int = 2;
    global_treckbar.push_treckbar("SorFilter: distance_to_neighbors", &distance_to_neighbors_int, 100);

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;
    sor_filter.setInputCloud(input_cloud);
    sor_filter.setMeanK(number_of_neighbors_int); // Количество соседей для анализа (например, 50)//20
    sor_filter.setStddevMulThresh((double)distance_to_neighbors_int / 100); // Удаляет точки, которые находятся дальше чем 1 стандартное отклонение от среднего расстояния //1.4
    sor_filter.filter(*output_cloud);

    return output_cloud;
}


PCLIptr mesh_struct::VoxelGridFilter(PCLIptr input_cloud)
{
    static int LeafSize_int = 6;
    global_treckbar.push_treckbar("VoxelGridFilter: LeafSize", &LeafSize_int, 40);

    //замена точек в вокселе одной точкой 
    // Step 2: Downsample using Voxel Grid Filter
    PCLIptr output_cloud(new PCLI);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize((float)LeafSize_int / 100, (float)LeafSize_int / 100, (float)LeafSize_int / 100);  // Adjust the leaf size as needed //0.1
    voxel_filter.filter(*output_cloud);

    return output_cloud;
}


PCLINptr mesh_struct::VoxelGridFilterNorm(PCLINptr input_cloud)
{
    static int LeafSize2_int = 30;
    global_treckbar.push_treckbar("VoxelGridFilter: LeafSize2", &LeafSize2_int, 1000);

    //замена точек в вокселе одной точкой 
    // Step 2: Downsample using Voxel Grid Filter
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::VoxelGrid<pcl::PointXYZINormal> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize((float)LeafSize2_int / 1000, (float)LeafSize2_int / 1000, (float)LeafSize2_int / 1000);  // Adjust the leaf size as needed //0.1
    voxel_filter.filter(*output_cloud);

    return output_cloud;
}


PCLINptr mesh_struct::Smoothing(PCLIptr input_cloud)
{   
    static int SearchRadius_int = 20;
    global_treckbar.push_treckbar("Smoothing: SearchRadius", &SearchRadius_int, 100);

    static int PolynomialOrder_int = 2;
    global_treckbar.push_treckbar("Smoothing: PolynomialOrder", &PolynomialOrder_int, 5);

    //сглаживание всех точек
    // Step 3: Smooth using Moving Least Squares
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZINormal> mls;  // Updated to output PointNormal
    mls.setInputCloud(input_cloud);
    mls.setSearchRadius((double)SearchRadius_int / 100);  // Set the smoothing radius //0.2
    mls.setPolynomialOrder(PolynomialOrder_int);  //2
    mls.setComputeNormals(true);  // Enable normal computation (needed for meshing)
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    mls.setSearchMethod(tree);
    mls.process(*output_cloud);

    return output_cloud;
}

pcl::PolygonMesh::Ptr mesh_struct::Meshing(PCLINptr input_cloud)
{
    static int MeshSearchRadius_int = 5;
    global_treckbar.push_treckbar("Meshing: MeshSearchRadius", &MeshSearchRadius_int, 200);

    static int Mu_int = 40;
    global_treckbar.push_treckbar("Meshing: Mu", &Mu_int, 100);

    static int MaximumNearestNeighbors_int = 110;
    global_treckbar.push_treckbar("Meshing: MaximumNearestNeighbors", &MaximumNearestNeighbors_int, 500);

    static int MaximumSurfaceAngle_int = 20;
    global_treckbar.push_treckbar("Meshing: MaximumSurfaceAngle", &MaximumSurfaceAngle_int, 100);

    static int MinimumAngle_int = 18;
    global_treckbar.push_treckbar("Meshing: MinimumAngle", &MinimumAngle_int, 100);

    static int MaximumAngle_int = 33;
    global_treckbar.push_treckbar("Meshing: MaximumAngle", &MaximumAngle_int, 100);
    //Создание сетки
    // Step 4: Create a mesh using Greedy Projection Triangulation
    pcl::GreedyProjectionTriangulation<pcl::PointXYZINormal> gp3;
    pcl::PolygonMesh::Ptr output_mesh(new pcl::PolygonMesh);
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree2->setInputCloud(input_cloud);
    // Set the parameters
    gp3.setSearchRadius((double)MeshSearchRadius_int / 100);  // Set the maximum distance between connected points (adjust as needed) 0.5
    gp3.setMu((double)Mu_int / 10);  // Multiplicative factor for search radius 2.5
    gp3.setMaximumNearestNeighbors(MaximumNearestNeighbors_int);  // Limit the number of nearest neighbors //100
    
    gp3.setMaximumSurfaceAngle(M_PI * ((double)MaximumSurfaceAngle_int / 100));  // Maximum angle (in radians) between surface normals // 16
    gp3.setMinimumAngle(M_PI * ((double)MinimumAngle_int / 100));  // Minimum angle for a triangle // 16
    gp3.setMaximumAngle(2 * M_PI * ((double)MaximumAngle_int / 100));  // Maximum angle for a triangle // 33
    gp3.setNormalConsistency(false);

    // Generate the meshя
    gp3.setInputCloud(input_cloud);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(*output_mesh);

    return output_mesh;
}

pcl::PolygonMesh::Ptr mesh_struct::Poisson_mesh(PCLINptr input_cloud)
{
    static int Depth_int = 8;
    global_treckbar.push_treckbar("Poisson_mesh: Depth", &Depth_int, 100);
    static int Scale_int = 0;
    global_treckbar.push_treckbar("Poisson_mesh: Scale", &Scale_int, 10);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::copyPointCloud(*input_cloud, *cloud_normals);

    pcl::Poisson<pcl::PointNormal> poisson;
    pcl::PolygonMesh::Ptr output_mesh(new pcl::PolygonMesh);
    poisson.setInputCloud(cloud_normals);
    poisson.setDepth(Depth_int);  // Отрегулируйте в зависимости от ваших данных
    poisson.setScale((float)Scale_int / 10 + 1);  // Отрегулируйте для гладкости
    poisson.reconstruct(*output_mesh);

    return output_mesh;
}

pcl::PolygonMesh::Ptr mesh_struct::FastMesh(PCLINptr input_cloud)
{
    static int TrianglePixelSize = 3;
    global_treckbar.push_treckbar("FastMesh: TrianglePixelSize", &TrianglePixelSize, 100);

    // Преобразование входного облака в pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_standart(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*input_cloud, *cloud_standart);

    // Проверяем, является ли облако точек организованным
    if (!cloud_standart->isOrganized())
    {
        std::cerr << "Input cloud is not organized. FastMesh requires an organized point cloud." << std::endl;
        return nullptr; // Возвращаем null, если облако не упорядочено
    }

    // Создаем объект OrganizedFastMesh
    pcl::OrganizedFastMesh<pcl::PointXYZ> fast_mesh;
    fast_mesh.setInputCloud(cloud_standart);
    fast_mesh.setTrianglePixelSize(TrianglePixelSize); // Настройка размера треугольников

    // Создаем объект PolygonMesh и передаем его в функцию reconstruct
    pcl::PolygonMesh::Ptr output_mesh(new pcl::PolygonMesh());
    fast_mesh.reconstruct(*output_mesh);

    // Возвращаем указатель на результат
    return output_mesh;
}

pcl::PolygonMesh::Ptr mesh_struct::Alpha_shapes(PCLINptr input_cloud)
{
    static int MeshSearchRadius_int = 5;
    global_treckbar.push_treckbar("Alpha: SearchRadius", &MeshSearchRadius_int, 200);

    static int Mu_int = 40;
    global_treckbar.push_treckbar("Alpha: Mu", &Mu_int, 100);

    static int MaximumNearestNeighbors_int = 110;
    global_treckbar.push_treckbar("Alpha: MaximumNearestNeighbors", &MaximumNearestNeighbors_int, 500);

    static int MaximumSurfaceAngle_int = 20;
    global_treckbar.push_treckbar("Alpha: MaximumSurfaceAngle", &MaximumSurfaceAngle_int, 100);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::copyPointCloud(*input_cloud, *cloud_normals);

    pcl::PolygonMesh::Ptr output_mesh(new pcl::PolygonMesh);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setSearchRadius((double)MeshSearchRadius_int / 100);  // Радиус поиска
    gp3.setMu((double)Mu_int / 10);             // Параметр влияния точек
    gp3.setMaximumNearestNeighbors(MaximumNearestNeighbors_int);  // Максимальное количество соседей
    gp3.setMaximumSurfaceAngle(M_PI * ((double)MaximumSurfaceAngle_int / 100)); // Максимальный угол поверхности
    gp3.setInputCloud(cloud_normals);
    gp3.reconstruct(*output_mesh);

    return output_mesh;
}


pcl::PolygonMesh::Ptr mesh_struct::MarchingCubes(PCLINptr input_cloud)
{
    static int GridResolution = 50;
    global_treckbar.push_treckbar("MarchingCubes: GridResolution", &GridResolution, 200);

    pcl::PolygonMesh::Ptr output_mesh(new pcl::PolygonMesh);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::copyPointCloud(*input_cloud, *cloud_normals);

    pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
    mc.setInputCloud(cloud_normals);
    mc.setGridResolution(GridResolution, GridResolution, GridResolution);  
    mc.reconstruct(*output_mesh);

    return output_mesh;
}


PCLIptr mesh_struct::convertToXYZI(const PCLINptr& input_cloud) 
{
    // Создаем новое облако для результата
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // Копируем данные из PointXYZINormal в PointXYZI
    for (const auto& point : *input_cloud) {
        pcl::PointXYZI new_point;
        new_point.x = point.x;
        new_point.y = point.y;
        new_point.z = point.z;
        new_point.intensity = point.intensity; // Копируем интенсивность
        output_cloud->points.push_back(new_point);
    }

    // Устанавливаем размеры и свойства облака
    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1; // Одномерное облако
    output_cloud->is_dense = input_cloud->is_dense;

    return output_cloud;
}

PCLINptr mesh_struct::computeNormals(PCLIptr cloud) 
{
    static int RadiusSearch_int = 50;
    global_treckbar.push_treckbar("ComputeNormals: MeshSearchRadius", &RadiusSearch_int, 1000);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);

    // Создание объекта для оценки нормалей
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(cloud);

    // Создаем KdTree для поиска ближайших соседей
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    normal_estimator.setSearchMethod(tree);

    // Устанавливаем радиус для оценки нормалей
    normal_estimator.setRadiusSearch((double)RadiusSearch_int / 100);

    // Вычисление нормалей
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    normal_estimator.compute(*normals);

    // Объединяем координаты и нормали
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    return cloud_with_normals;
}

void mesh_struct::write_point_xyz(pcl::visualization::PCLVisualizer* viewer)
{
    // Ограничим количество подписанных точек
    int id = 0;
    int step = std::max(1, static_cast<int>(cloudXYZI_viz->points.size() / 1000)); // Подписывать только 100 точек
    for (size_t i = 0; i < cloudXYZI_viz->points.size(); i += step) 
    {
        const auto& point = cloudXYZI_viz->points[i];
        std::string text = "(" + std::to_string(point.x).substr(0, 7) + ", " +
                            std::to_string(point.y).substr(0, 7) + ")";// + ", " +
                            //std::to_string(point.z).substr(0, 5) + ")";
        viewer->addText3D(text, point, 0.001, 1.0, 1.0, 1.0, "text" + std::to_string(id));
        id++;
    }
}

void mesh_struct::fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) 
{
    // Вычисляем центр облака точек
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input_cloud, centroid);
    
    // Вычисляем PCA (анализ главных компонент)
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(input_cloud);
    
    // Получаем вектор нормали (наименьшее собственное значение)
    Eigen::Vector3f normal = pca.getEigenVectors().col(2);

    // Вычисляем d в уравнении плоскости ax + by + cz + d = 0
    float d = -normal.dot(centroid.head<3>());

    // Выводим уравнение плоскости
    /*std::cout << "Equation of plane: " << normal[0] << "x + "
              << normal[1] << "y + " << normal[2] << "z + "
              << d << " = 0" << std::endl;*/
}

std::vector<float> mesh_struct::Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(1.01); // Допустимое расстояние до плоскости

    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
        std::cerr << "Плоскость не найдена!" << std::endl;
    
    std::vector<float> coef = {coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]};

    //std::cout << "Уравнение плоскости: " << coef[0] << " * x + " << coef[1] << " * y + "<< coef[2] << " * z + "<< coef[3] << " = 0" << std::endl;
    return coef;
}

PCLIptr mesh_struct::generatePlane(std::vector<float> coef, float range)
{
    PCLIptr output_cloud(new PCLI);
    float a = coef[0], b = coef[1], c = coef[2], d = coef[3];
    float step = 0.02;
    for (float z = -range; z <= range; z += step)
        for (float y = -range; y <= range; y += step)
        {
            float x = (-b*y -c*z - d) / a; 
            //std::cout << "x: " << x << "y:"<< y << "z:"<< z << std::endl;
            pcl::PointXYZI point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = 100;
            output_cloud->points.push_back(point);
        }
    return output_cloud;
}


float mesh_struct::count_normal_error(std::vector<float> coef, PCLINptr input_cloud)
{
    PCLIptr output_cloud(new PCLI);
    float a = coef[0], b = coef[1], c = coef[2], d = coef[3];
    float summ = 0;
    for (unsigned int idx = 0; idx < input_cloud->size(); ++idx)
    {
        float x = input_cloud->at(idx).x;
        float y = input_cloud->at(idx).y;
        float z = input_cloud->at(idx).z;

        //std::cout << "x: " << x << "y:"<< y << "z:"<< z << std::endl;
        float norm_error = (abs(a*x + b*y + c*z + d) / sqrt(a*a+b*b+c*c));
        summ += norm_error;
    }

    return (summ / input_cloud->size());
}