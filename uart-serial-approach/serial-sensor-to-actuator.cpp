#include <CppLinuxSerial/SerialPort.hpp>

// This code uses the following library: 
// https://github.com/gbmhunter/CppLinuxSerial
//
// Compiling (when CppLinuxSerial is installed on the system):
// g++ serial-sensor-to-actuator.cpp -lCppLinuxSerial

using namespace mn::CppLinuxSerial;

int main(){
    SerialPort serSensor("/dev/ttyACM0", BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    SerialPort serActuator("/dev/ttyACM1", BaudRate::B_460800, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serSensor.SetTimeout(-1);
    serActuator.SetTimeout(-1);
    serSensor.Open();
    serActuator.Open();

    std::string input;

    std::cout << "Waiting for first sensor value" << std::endl;
    serSensor.Read(input);
    std::cout << input << std::endl;

    while(true){
        serSensor.Read(input);
        serActuator.Write(input);
        //std::cout << "input = " << input << std::endl;
        while(serActuator.Available() > 0){
            serActuator.Read(input);
            std::cout << "Actuator: " << input << std::endl;
        }
        
    }

    serSensor.Close();
    serActuator.Close();
}