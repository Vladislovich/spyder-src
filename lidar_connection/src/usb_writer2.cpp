#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>

using namespace boost::asio;
using namespace std;

atomic<bool> run_input_thread(true);

void set_nonblocking_input_mode() {
    termios tty;
    tcgetattr(STDIN_FILENO, &tty);

    // Disable canonical mode and echo
    tty.c_lflag &= ~(ICANON | ECHO);

    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

void restore_input_mode() {
    termios tty;
    tcgetattr(STDIN_FILENO, &tty);

    // Re-enable canonical mode and echo
    tty.c_lflag |= (ICANON | ECHO);

    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

void input_thread(serial_port &serial) {
    while (run_input_thread) {
        char input;
        if (read(STDIN_FILENO, &input, 1) > 0) { // Read a single character
            if (isdigit(input)) {
                string message(1, input);
                message += "\n";
                boost::asio::write(serial, boost::asio::buffer(message));
                cout << "Sent to Arduino: " << input << endl;
            } else {
                cerr << "Invalid input. Please enter a digit." << endl;
            }
        }
    }
}

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

    cout << "Listening for single digit input and communicating with Arduino on " << port_name << "..." << endl;

    try {
        boost::asio::streambuf buffer;
        set_nonblocking_input_mode();

        thread input_thread_instance(input_thread, ref(serial));

        while (true) {
            size_t bytes = serial.read_some(buffer.prepare(128));
            if (bytes > 0) {
                buffer.commit(bytes);
                istream is(&buffer);
                string data;
                while (getline(is, data)) {
                    cout << "Received: " << data << endl;
                }
            }
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    } catch (boost::system::system_error &e) {
        cerr << "Error: " << e.what() << endl;
    }

    restore_input_mode();
    run_input_thread = false;
    return 0;
}
