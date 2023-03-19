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

### Installing `sockpp`:
`sockpp` is a high-level socket library for C++.

```
git clone https://github.com/fpagliughi/sockpp.git
cd sockpp
mkdir build
cd build
cmake ..
cmake --build . --target install
```

### Installing `CppLinuxSerial`:
`'CppLinuxSerial` is a library to access the serial ports in C++.

```
git clone https://github.com/gbmhunter/CppLinuxSerial.git
cd CppLinuxSerial
mkdir build
cd build
cmake ..
make
sudo make install
```

## Compiling the Source:
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

## Running the Executables:
### Explanations:
The `pendulum_sender` and `cross_traffic` executable must be run with root privileges. Otherwise, the PCP value `7` is not allowed by the OS and PCP value `0` is used instead (without reporting an error).

You need to make sure that the shared libraries (`sockpp` and `CppLinuxSerial`) can be found. For this purpose, you need to add the path of the libraries (usually `/usr/local/lib`) to the `LD_LIBRARY_PATH` variable:
```
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### Running Pendulum Sender:
First connect the Teensy to the single-board microcontroller via USB. Then run
```
sudo su
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
./pendulum_sender
```

### Running Cross Traffic Sender:
```
sudo su
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
./cross_traffic
```

### Running Pendulum Receiver:
First connect the Teensy to the single-board microcontroller via USB. Then run
```
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
./pendulum_sender
```


