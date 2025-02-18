#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>
#include <iostream>

int background_r = 0, background_g = 0, background_b = 0; // Цвет фона
int search_radius = 5; // Радиус поиска для сглаживания (умноженный на 10 для точности)

void on_trackbar(int, void*) {
    // Пустая функция для обработки события трекбара (можно оставить пустой)
}

int main() {
    // Загрузите ваш mesh файл
    pcl::PolygonMesh mesh;
    std::string file_path = "../points_and_meshes/mesh_smooth.ply";
    if (pcl::io::loadPLYFile(file_path, mesh) == -1) {
        std::cerr << "Ошибка: не удалось загрузить файл " << file_path << std::endl;
        return -1;
    }
    std::cout << "Файл " << file_path << " загружен успешно" << std::endl;

    // Создайте визуализатор PCL
    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0); // Начальный черный фон
    viewer.addPolygonMesh(mesh, "mesh");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

    // Создайте окно для трекбаров
    cv::namedWindow("Controls", cv::WINDOW_AUTOSIZE);

    // Добавьте трекбары для настройки цвета фона
    cv::createTrackbar("Background R", "Controls", &background_r, 255, on_trackbar);
    cv::createTrackbar("Background G", "Controls", &background_g, 255, on_trackbar);
    cv::createTrackbar("Background B", "Controls", &background_b, 255, on_trackbar);

    // Трекбар для радиуса поиска
    cv::createTrackbar("Search Radius (x0.1)", "Controls", &search_radius, 100, on_trackbar);

    while (!viewer.wasStopped()) {
        // Обновите цвет фона на основе значений трекбаров
        viewer.setBackgroundColor(background_r / 255.0, background_g / 255.0, background_b / 255.0);

        // Применяйте радиус поиска к сглаживанию, если требуется
        float actual_radius = search_radius / 10.0f; // Текущий радиус из трекбара
        std::cout << "Текущий радиус поиска: " << actual_radius << std::endl;

        viewer.spinOnce(100);
        cv::waitKey(1); // Задержка, чтобы OpenCV окно могло обновиться
    }

    return 0;
}