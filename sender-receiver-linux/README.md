# Pendulum Sender, Receiver and Cross-Traffic Sender for Linux
This CMake project contains the code for following programs:
- Sender component for the sensor-side single-board computer
- Receiver component for the actuator-side single-board computer
- Cross-traffic generator

All the code is written in C++. The following libraries are used:
- `sockpp` to open UDP sockets
- `CppLinuxSerial` to interact with the USB serial interface
- `nlohmann/json` to generate the JSON log files

`sockpp` and `CppLinuxSerial` are shared libraries: They must be installed on the system that is supposed to run the program. They are not included in the executable.
`nlohmann/json` is included in the source code of this project. Thus, the library is included in the executable.

## Preparation
### Installing CMake
Unfortunately, the APT repository only has an outdated version of CMake at the time of writing. Therefore, `sudo apt install cmake` doesn't work.

Instead, use one of the two options:
- Install via Snap:  `sudo snap install cmake --classic`
- Install executable from https://cmake.org/download/

To use CMake, you need to install `build-essential` too:
```
sudo apt install build-essential
```

### Installing `sockpp`
`sockpp` is a high-level socket library for C++.

```
git clone https://github.com/fpagliughi/sockpp.git
cd sockpp
mkdir build
cd build
cmake ..
sudo cmake --build . --target install
```

### Installing `CppLinuxSerial`
`CppLinuxSerial` is a library to access the serial ports in C++.

```
git clone https://github.com/gbmhunter/CppLinuxSerial.git
cd CppLinuxSerial
mkdir build
cd build
cmake ..
make
sudo make install
```

## Compiling the Source
```
mkdir cmake-build-debug
cd cmake-build-debug
cmake ..
cmake --build .
```

To compile the different targets individually, you can also run instead of the last command:
```
cmake --build . --target pendulum_sender
cmake --build . --target pendulum_receiver
cmake --build . --target cross_traffic
```

## Running the Executables
### Explanations
The `pendulum_sender` and `cross_traffic` executable must be run with root privileges. Otherwise, the PCP value `7` is not allowed by the OS and PCP value `0` is used instead (without reporting an error).

You need to make sure that the shared libraries (`sockpp` and `CppLinuxSerial`) can be found. For this purpose, you need to add the path of the libraries (usually `/usr/local/lib`) to the `LD_LIBRARY_PATH` variable:
```
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### Running Pendulum Sender
First connect the Teensy to the single-board microcontroller via USB. Then run
```
sudo su
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
./pendulum_sender
```

### Running Cross Traffic Sender
```
sudo su
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
./cross_traffic
```

### Running Pendulum Receiver
First connect the Teensy to the single-board microcontroller via USB. Then run
```
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
./pendulum_sender
```

## Note About Connecting Teensy via USB
Sometimes, the serial socket doesn't work when the Teensy has been plugged in for a longer time.
In this case, there simply is no input from the serial interface, no errors.

To avoid this problem:
1. Unplug the Teensy
2. Plug in the Teensy
3. Launch the program on the single-board computer immediately (max 10 seconds after plugging in)


## Starting Configurations
The sender and receiver can be started with various parameter options:

### Starting the Sender
#### With single-run config file
````
./pendulum_sender f config_sender_single.json
````

#### With sequence config file
````
./pendulum_sender s config_sender_sequence.json
````

### Starting the Receiver
````
./pendulum_receiver config_receiver.json
````

## Configuration Files
### Sender Config (single run)
```json
{
  "b": 200.0,
  "r": 35.0,
  "numThresholds": 3,
  "thresholds": [0, -200, -400],
  "prioMapping": [0, 1, 2, 7],
  "costs": [1, 1, 1, 0],
  "historySize": 100,
  "bias": 0,
  "samplingPeriods": [100, 90, 80, 70, 60, 50, 40, 30, 20, 10],
  "samplingPeriodSensitivityFactor": 1.0,
  "samplingPeriodSensitivityOffset": 0.0,
  "initialPriorityClass": 0,
  "serialDeviceName": "auto",
  "receiverAddress": "10.0.1.3",
  "networkDelaysPerPrio": [2, 4, 6, 8, 10, 12, 14, 16]
}
```

with
- `b`: Bucket size of the token bucket
- `r`: Refill rate of the token bucket
- `numThresholds`: Number of thresholds given by the `thresholds` array.
- `thresholds`: Array of thresholds (floating point numbers) for the priority classes. The lowest threhold `- infinity` is ommited from the array.
- `prioMapping`: Array specifying the mapping from the thresholds to the priority classes (integers 0-7).
- `costs`: Array of costs associated with the priority classes. First element associated with best priority class.
- `historySize`: Number of samples kept in the sensor history to determine the maximum absolute angle change.
- `bias` Bias as an integer value. Is *added* to the raw sensor value before sending it to the receiver. Default value: `0`
- `samplingPeriods`: Array of 10 sampling periods in milliseconds, where the first element corresponds to the smallest change and the 10th element corresponds to the largest maximum absolute angle change.
- `initialPriorityClass`: Priority class used at start ()
- `samplingPeriodSensitivityFactor`: Factor by which the maximum absolute angle change is multiplied to determine sampling period. Default value: `1.0`
- `samplingPeriodSensitivityOffset`: Offset by which the maximum absolute angle change is shifted to determine sampling period. Default value: `0.0`
- `serialDeviceName`: Linux file which associated with the serial interface of the sender Teensy. If you set it to `auto`, the computer tries to recognize a Teensy device automatically. Default value: `auto`
- `receiverAddress`: IP address of the receiver PC
- `networkDelaysPerPrio`: Array of network delays in milliseconds (integers) for each priority class. Must have 8 elements. First element associated with the best priority class (0).


### Sender Config (multiple MPTB configurations)
This config allows to run multiple MPTB configurations one after another without manual interaction.
```json
{
  "mptbSequence": [
    {
      "name": "mptb1",
      "durationMinutes": 1.0,
      "b": 200.0,
      "r": 35.0,
      "numThresholds": 3,
      "thresholds": [-20000000, -200, -400],
      "prioMapping": [0, 1, 2, 7],
      "costs": [1, 1, 1, 0]
    },
    {
      "name": "mptb2",
      "durationMinutes": 0.5,
      "b": 5000.0,
      "r": 350.0,
      "numThresholds": 2,
      "thresholds": [-20000, -40000],
      "prioMapping": [0, 1, 2],
      "costs": [2, 1, 0]
    }
  ],
  "historySize": 100,
  "bias": 0,
  "samplingPeriods": [100, 90, 80, 70, 60, 50, 40, 30, 20, 10],
  "samplingPeriodSensitivityFactor": 1.0,
  "samplingPeriodSensitivityOffset": 0.0,
  "serialDeviceName": "auto",
  "receiverAddress": "10.0.1.5",
  "networkDelaysPerPrio": [2, 4, 6, 8, 10, 12, 14, 16]
}

```

with
- `name`: Name of the configuration. This name is used for the file name of the log file.
- `mptbSequence`: Array with an arbitrary number of MPTB configuration objects. These configurations are executed sequentially in the order given by the array.
- `durationMinutes`: Number of minutes (floating point number) that this configuration should be executed.
- All other parameters like in the single-run config.

### Receiver Config
```json
{
  "receiverAddress": "10.0.1.3",
  "pendulumType": "oldPendulum",
  "serialDeviceName": "auto",
  "swingUpBehavior": "crashAndSwingUpAtNewConfig",
  "sailType": "sail14",
  "controllerKVector": [1.0, 2.0, 3.0, 4.0],
  "controllerIntegratorParam": 1.0,
  "controlApproach":"carabelliKalmanCarabelliController"
}
```

with 
- `receiverAddress`: IP address of the receiver PC (i.e., own IP address). This IP address is used to bind the UDP socket.
- `pendulumType`: Type of the pendulum. Influences the motor characteristics. Possible options:
    - `oldPendulum`: The pendulum constructed by Ben Carabelli
    - `newPendulum`: The two pendulums constructed by David Augustat
- `serialDeviceName`: Linux file which associated with the serial interface of the receiver Teensy. If you set it to `auto`, the computer tries to recognize a Teensy device automatically. Default value: `auto`
- `swingUpBehavior`: Determines if and when the pendulum should swing up itself. Default: `noSwingUp`. Possible options:
    - `swingUpAtStart`: Only swings up the pendulum at the start. If the pendulum crashes, it will stay crashed.
    - `swingUpAtNewConfigIfCrashed`: Swings up the pendulum at start and at the beginning of each new config, **if the pendulum has crashed before**. If the pendulum is already balancing at the start of a new config, it will not swing up.
    - `crashAndSwingUpAtNewConfig`: Swings up the pendulum at the start. At the beginning of each new config, the pendulum intenionally stops balancing to enforce a crash. Then the pendulum swings up again.
    - `noSwingUp`: Manual mode where the pendulum must be held upwards by the operating person at the beginning. No swing-up.
- `sailType`: Determines which sail is used. Default: `noSail`. Possible options:
    - `noSail`: No sail is used.
    - `sail10`
    - `sail14`
    - `sail17`
    - `sail20`
- `controllerKVector`: Vector of 4 floating point numbers. The IST controller uses this vector as the K vector. Default: `[3.6723, 13.5022, -74.6153, -19.8637]`
- `controllerIntegratorParam`: Floating point number. The IST controller uses this value as the integrator parameter. Default: `5.0322`
- `controlApproach`: String. Determines which control approach is used. Default: `carabelliKalmanCarabelliController`. Possible options:
    - `carabelliKalmanCarabelliController`: use the Kalman filter and the controller from Carabelli
    - `carabelliKalmanIstController`: use the Kalman filter from Carabelli and the controller from IST
