#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/streambuf.hpp>
#include <thread>
#include <atomic>

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

    cout << "Listening for data from Arduino on " << port_name << "..." << endl;

    try {
        boost::asio::streambuf buffer;
        this_thread::sleep_for(std::chrono::milliseconds(10000));
        while (true) {
            size_t bytes = serial.read_some(buffer.prepare(128));
            if (bytes > 0) {
                buffer.commit(bytes);
                istream is(&buffer);
                string data;
                while (getline(is, data)) {
                    double value = std::strtod(data.c_str(), nullptr);
                    cout << "Received: " << value << endl;
                }
                buffer.consume(bytes);
            }
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    } catch (boost::system::system_error &e) {
        cerr << "Error: " << e.what() << endl;
    }

    return 0;
}
