#include <CppLinuxSerial/SerialPort.hpp>
#include <chrono>
#include <thread>

// This code uses the following library: 
// https://github.com/gbmhunter/CppLinuxSerial
//
// Compiling (when CppLinuxSerial is installed on the system):
// g++ serial-latency-test-host.cpp -lCppLinuxSerial


using namespace mn::CppLinuxSerial;

int main(){
    SerialPort serialPort("/dev/ttyACM0", BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(-1);
    serialPort.Open();

    std::string input;

    // Start:
    serialPort.Write("Start!\n");
    serialPort.Read(input);
    std::cout << "Received: " << input << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while(1){
        serialPort.Read(input);
        if(input[0] == 'T'){
            serialPort.Write(input);
        } else {
            std::cout << "Teensy: " << input << std::endl;
        }
    }

    serialPort.Close();
}
