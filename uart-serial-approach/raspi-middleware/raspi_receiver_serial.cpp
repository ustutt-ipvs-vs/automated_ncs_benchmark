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
// g++ raspi_receiver_serial.cpp -lCppLinuxSerial -lsockpp

std::string receiverHost = "127.0.0.1";
int receiverPort = 3000;
char receiveBuffer[1500];
std::string serialInput;
std::string networkInput;

int main(){
    // Set up serial for actuator:
    SerialPort serActuator("/dev/ttyACM1", BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serActuator.SetTimeout(-1);
    serActuator.Open();

    // Set up receiving UDP socket:
    sockpp::udp_socket receiverSocket;
    sockpp::inet_address receiverAddress(receiverHost, receiverPort);
    receiverSocket.bind(receiverAddress);

    while(true){
        int receivedLength = receiverSocket.recv(receiveBuffer, sizeof(receiveBuffer));
        networkInput = std::string(receiveBuffer, receivedLength);

        std::cout << "Received from network: " << networkInput << std::endl;
        serActuator.Write(networkInput);

        while(serActuator.Available() > 0){
            serActuator.Read(serialInput);
            std::cout << "Actuator: " << serialInput << std::endl;
        }
    }

    serActuator.Close();
}
