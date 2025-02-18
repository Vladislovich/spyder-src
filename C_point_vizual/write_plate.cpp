#include <iostream>
#include <fstream>
#include <cstdlib>  // Для rand() и srand()
#include <ctime> 

int main() {
    // Открываем файл для записи
    std::ofstream outFile("../points_and_meshes/pointcloud_log_6_edit.txt");

    if (!outFile.is_open()) {
        std::cerr << "Не удалось открыть файл для записи!" << std::endl;
        return 1;
    }
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    for (int i = 0; i < 100; i++)
        for (int j = 0; j < 100; j++)
        {
            // Генерация случайного числа от 0 до 1
            double randomValue = static_cast<double>(std::rand()) / RAND_MAX;

            // Умножение на 0.1 для получения числа от 0 до 0.1
            double randomNumber = randomValue * 0.1;
            outFile << std::to_string((double)i / 100) + " " + std::to_string(randomNumber) + " " + std::to_string((double)j / 100) + " 100" << std::endl;
        }

    // Закрываем файл
    outFile.close();

    std::cout << "Данные успешно записаны в файл output.txt" << std::endl;

    return 0;
}