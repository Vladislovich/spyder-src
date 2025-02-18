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

    cout << "Listening for input from command line and communicating with Arduino on " << port_name << "..." << endl;

    try {
        boost::asio::streambuf buffer;
        while (true) {
            // Проверка ввода из командной строки
            if (cin.peek() != EOF) {
                int input;
                cin >> input;
                if (!cin.fail()) {
                    // Отправка целого числа на Arduino
                    string message = to_string(input) + "\n";
                    boost::asio::write(serial, boost::asio::buffer(message));
                    cout << "Sent to Arduino: " << input << endl;
                } else {
                    cin.clear(); // Сброс ошибки
                    cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Очистка ввода
                    cerr << "Invalid input. Please enter an integer." << endl;
                }
            }

            // Получение данных от Arduino
            size_t bytes = serial.read_some(buffer.prepare(128));
            if (bytes > 0) {
                buffer.commit(bytes);
                istream is(&buffer);
                string data;
                while (getline(is, data)) {
                    cout << "Received: " << data << endl;
                }
            }
        }
    } catch (boost::system::system_error &e) {
        cerr << "Error: " << e.what() << endl;
    }

    return 0;
}
