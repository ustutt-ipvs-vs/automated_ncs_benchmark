#include <CppLinuxSerial/SerialPort.hpp>
#include <sockpp/udp_socket.h>

using mn::CppLinuxSerial::SerialPort;
using mn::CppLinuxSerial::BaudRate;
using mn::CppLinuxSerial::NumDataBits;
using mn::CppLinuxSerial::Parity;
using mn::CppLinuxSerial::NumStopBits;
using sockpp::udp_socket;
using sockpp::inet_address;

// This code uses the following libraries: 
// https://github.com/gbmhunter/CppLinuxSerial
// https://github.com/fpagliughi/sockpp
// 
// Setup sockpp as shared library by running:
// LD_LIBRARY_PATH=/usr/local/lib
// export LD_LIBRARY_PATH
//
// Compiling (when both libraries are installed on the system):
// g++ raspi_sender_serial.cpp -lCppLinuxSerial -lsockpp

std::string receiverHost = "127.0.0.1";
int receiverPort = 3000;

std::string input;

int main(){
    // Set up serial input for sensor:
    SerialPort serSensor("/dev/ttyACM0", BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serSensor.SetTimeout(-1);
    serSensor.Open();

    // Set up UDP socket:
    udp_socket senderSocket;
    inet_address receiverAddress(receiverHost, receiverPort);

    // Wait for first serial values to arrive, before going into main loop:
    std::cout << "Waiting for first sensor value" << std::endl;
    serSensor.Read(input);
    std::cout << input << std::endl;

    while(true){
        serSensor.Read(input);
        std::cout << "input = " << input << std::endl;
        senderSocket.send_to(input, receiverAddress);
    }

    serSensor.Close();
}
