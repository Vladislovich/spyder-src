#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/streambuf.hpp>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>

using namespace boost::asio;
using namespace std;

atomic<bool> start_reading(false);  // Флаг, который определяет, когда начинать читать данные

void set_nonblocking_input_mode() {
    termios tty;
    tcgetattr(STDIN_FILENO, &tty);

    // Отключаем каноничный режим и эхо
    tty.c_lflag &= ~(ICANON | ECHO);

    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

void restore_input_mode() {
    termios tty;
    tcgetattr(STDIN_FILENO, &tty);

    // Включаем каноничный режим и эхо
    tty.c_lflag |= (ICANON | ECHO);

    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

void input_thread() {
    while (true) {
        char input;
        if (read(STDIN_FILENO, &input, 1) > 0) { // Чтение одного символа
            if (input == '2') {
                cout << "Received '2', starting to read from serial port..." << endl;
                start_reading = true; // Устанавливаем флаг для начала чтения
                break;  // Выходим из цикла, чтобы начать чтение
            }
        }
    }
}

int main() {
    // Настройка Boost.Asio
    io_service io;

    // Укажите порт вашего Arduino (например, "/dev/ttyUSB0" на Linux или "COM3" на Windows)
    string port_name = "/dev/arduino";
    serial_port serial(io, port_name);

    // Настройка параметров порта
    serial.set_option(serial_port_base::baud_rate(115200));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

    cout << "Waiting for key press '2' to start reading from Arduino on " << port_name << "..." << endl;

    try {
        set_nonblocking_input_mode();

        while (true) {
            // Запускаем отдельный поток для ожидания нажатия клавиши '2'
            thread input_thread_instance(input_thread);

            // Ожидаем нажатия клавиши '2'
            input_thread_instance.join();

            // После нажатия '2' начинаем читать данные с серийного порта
            if (start_reading) {
                boost::asio::streambuf buffer;
                cout << "Started reading from serial port..." << endl;

                // Чтение одного числа
                size_t bytes = serial.read_some(buffer.prepare(128)); // Чтение данных
                if (bytes > 0) {
                    buffer.commit(bytes);  // Закрепляем прочитанные данные в буфере
                    istream is(&buffer);
                    string data;
                    if (getline(is, data)) {  // Считываем строку
                        double value = std::strtod(data.c_str(), nullptr);  // Преобразуем строку в число
                        cout << "Received number: " << value << endl;
                    } else {
                        cout << "No valid number received." << endl;
                    }
                }
                // После получения данных, программа снова ждет нажатия '2'
                start_reading = false;
            }
        }

    } catch (boost::system::system_error &e) {
        cerr << "Error: " << e.what() << endl;
    }

    restore_input_mode();
    return 0;
}
