#include <iostream>
#include <boost/asio.hpp>

using namespace boost::asio;
using namespace std;

int main() {
    // Настройка Boost.Asio
    io_service io;

    // Замените "/dev/ttyUSB0" на порт вашей Arduino (например, "COM3" на Windows)
    string port_name = "/dev/arduino";
    serial_port serial(io, port_name);

    // Настройка параметров порта
    serial.set_option(serial_port_base::baud_rate(115200));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

    cout << "Listening for data on " << port_name << "..." << endl;

    try {
        while (true) {
            char c;
            boost::asio::read(serial, buffer(&c, 1)); // Читаем по одному символу

            // Если получили символ новой строки, выводим накопленные данные
            if (c == '\n') {
                cout << endl;
            } else {
                cout << c; // Выводим данные в консоль
            }
        }
    } catch (boost::system::system_error &e) {
        cerr << "Error: " << e.what() << endl;
    }

    return 0;
}
