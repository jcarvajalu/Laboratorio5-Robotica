cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_one_motor)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_controllers
  roscpp
  rospy
  std_msgs
)
install(FILES
  "src/dynamixel_one_motor/anexos/pose1.png"
  "src/dynamixel_one_motor/anexos/pose2.png"
  "src/dynamixel_one_motor/anexos/pose3.png"
  "src/dynamixel_one_motor/anexos/pose4.png"
  "src/dynamixel_one_motor/anexos/poshome.png"
  "src/dynamixel_one_motor/anexos/def.png"
  "src/dynamixel_one_motor/anexos/stop.png"

  "src/dynamixel_one_motor/anexos/trayF1.csv"
  "src/dynamixel_one_motor/anexos/trayF2.csv"
  "src/dynamixel_one_motor/anexos/trayG1.csv"
  "src/dynamixel_one_motor/anexos/trayG2.csv"
  "src/dynamixel_one_motor/anexos/trayN1.csv"
  "src/dynamixel_one_motor/anexos/trayN2.csv"
  "src/dynamixel_one_motor/anexos/trayH1.csv"
  "src/dynamixel_one_motor/anexos/trayH21.csv"
  "src/dynamixel_one_motor/anexos/trayH22.csv"
  "src/dynamixel_one_motor/anexos/trayH23.csv"
  "src/dynamixel_one_motor/anexos/trayWI1.csv"
  "src/dynamixel_one_motor/anexos/trayWI2.csv"
  "src/dynamixel_one_motor/anexos/trayWO1.csv"
  "src/dynamixel_one_motor/anexos/trayWO2.csv"
  "src/dynamixel_one_motor/anexos/trayHomecsv"

  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/anexos
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamixel_one_motor
# CATKIN_DEPENDS dynamixel_workbench_controllers roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  #include
  src
  anexos
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/jointSrv.py
  scripts/jointSub.py
  scripts/jointPub.py
  scripts/phantomHMI.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


