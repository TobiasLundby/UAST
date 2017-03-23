cmake_minimum_required(VERSION 2.8.3)
project(simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  cv_bridge
  image_transport
  sensor_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

add_executable(MarkerLocater src/markerLocator.cpp src/ImgConverter.cpp)
target_link_libraries(MarkerLocater ${catkin_LIBRARIES})

add_executable(NeuralControler src/neural.cpp)
target_link_libraries(NeuralControler ${catkin_LIBRARIES})

add_executable(XboxController src/xboxController.cpp src/ImgConverter.cpp)
target_link_libraries(XboxController ${catkin_LIBRARIES} -lbinreloc -pthread -lexpat -lgtkmm-2.4 -latkmm-1.6 -lgdkmm-2.4 -lgiomm-2.4 -lpangomm-1.4 -lgtk-x11-2.0 -lglibmm-2.4 -lcairomm-1.0 -lgdk-x11-2.0 -latk-1.0 -lgio-2.0 -lpangoft2-1.0 -lpangocairo-1.0 -lgdk_pixbuf-2.0 -lcairo -lpango-1.0 -lfontconfig -lgobject-2.0 -lglib-2.0 -lfreetype -lsigc-2.0 -lX11)

target_include_directories(XboxController PUBLIC
  /home/stagsted/RMUAST/src/simulation
  /usr/include/glibmm-2.4
  /usr/lib/x86_64-linux-gnu/glibmm-2.4/include
  /usr/include/glib-2.0
  /usr/lib/x86_64-linux-gnu/glib-2.0/include
  /usr/include/sigc++-2.0
  /usr/lib/x86_64-linux-gnu/sigc++-2.0/include
)
