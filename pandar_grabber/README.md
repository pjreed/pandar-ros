## Prerequisite
1. libpcap-dev
  On Ubuntu, `sudo apt-get install libpcap-dev`
2. pcl >= 1.7
3. protocal buffer
  On Ubuntu, `sudo apt-get install protobuf-compiler libprotobuf-dev`


## Usage
1. Link to this library  
  Include this project as subdirectory in your CMakeLists.txt:  
  ```cmake
  add_subdirectory(pandar_grabber)
  
  add_executable(hello_panda xxx.cpp)
  target_link_libraries(hello_panda
  	padar_grabber
  	)
  ```
2. Using [PandarGrabber](include/pandar_grabber/pandar_grabber.h) in your program  
  a.Iinclude header file
  ```c++
  #include "pandar_grabber/pandar_grabber.h"
  ```

  b. Instantiate a PandarGrabber with proper parameters
  ```c++
  // read pcap file
  pcl::PandarGrabber my_grabber(calibration_file, pcapFile);
  // or read ethernet
  // pcl::PandarGrabber my_grabber(ip, port, calibration_file);
  ```

  c. Register point cloud handler
  ```c++
  boost::signals2::connection c = interface.registerCallback(f);
  // f is a function pointer or a boost::funtion
  ```

  d. Start grabber
  ```c++
  my_grabber.start();
  ```

