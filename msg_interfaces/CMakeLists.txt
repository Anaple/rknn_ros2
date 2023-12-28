cmake_minimum_required(VERSION 3.5)
project(msg_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#添加对sensor_msgs的
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#添加消息文件和依赖
rosidl_generate_interfaces(${PROJECT_NAME}
  # "navigation_srv_interfaces/GobalNavigationInterface.srv"
  # "navigation_msg_interfaces/LocalNavigationInterface.msg"
  "navigation_msg_interfaces/GolbalPathPalnningInterface.msg"
  # "navigation_msg_interfaces/TrafficMessageInterface.msg"
  # "navigation_msg_interfaces/LocalPathPalnningInterface.msg"

  # "runtime_msg_interfaces/ComReadInterface.msg"
  # "runtime_msg_interfaces/ComSendInterface.msg"
  "runtime_msg_interfaces/EmergencyInterface.msg"
  # "runtime_msg_interfaces/PidInterfaces.msg"
  # "runtime_msg_interfaces/MotionControlInterface.msg"

  # "sensor_msg_interfaces/BatteryInterface.msg"
  
  # "sensor_msg_interfaces/DistanceInterface.msg"
  # "sensor_msg_interfaces/ImuInterface.msg"
  # "sensor_msg_interfaces/MagneticInterface.msg"
  # "sensor_msg_interfaces/RfidInterface.msg"
  # "sensor_msg_interfaces/SensorFusionInterface.msg"
  # "sensor_msg_interfaces/ObsInterface.msg"
  # "sensor_msg_interfaces/SonicObstacleInterface.msg"


  "network_msg_interfaces/RelaySwitchInterface.msg"
  "network_msg_interfaces/NavSelectInterface.msg"
  # "network_msg_interfaces/TrafficLightMessagesInterface.msg"

  # "auto/Decison.msg"
  # "auto/Fusion.msg"
  # "auto/NetDevice.msg"
  # "auto/TrafficLight.msg"
  # "auto/RGBCameraLane.msg"
  # "auto/RGBCameraLights.msg"
  "auto/RGBCameraObstacle.msg"
  # "auto/RGBCameraSigns.msg"
  
  DEPENDENCIES sensor_msgs

  DEPENDENCIES std_msgs
 )
#"srv/GlobalPathPlanningInterface.srv"

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
