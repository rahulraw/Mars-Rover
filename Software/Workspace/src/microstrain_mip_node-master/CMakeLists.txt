cmake_minimum_required(VERSION 2.8)
project(microstrain_mip_node)


find_package(catkin REQUIRED COMPONENTS
   roscpp
   sensor_msgs
   std_msgs
   geometry_msgs
   tf
)


add_service_files(
   DIRECTORY srv  
   FILES AddOffset.srv
)


generate_messages()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES $(PROJECT_NAME)
   CATKIN_DEPEND roscpp sensor_msgs self_test tf geometry_msgs diagnostic_msgs std_srvs
)


include_directories(
   include 
   ${catkin_INCLUDE_DIRS}
)

file(GLOB MIP_SDK_SRC
   "src/MIP_SDK/*.cpp"
)

add_library(MIP_SDK ${MIP_SDK_SRC})

add_executable(GX3_35_Test src/MIP_SDK/GX3_35_Test.cpp)

target_link_libraries(GX3_35_Test MIP_SDK)

add_executable(driver src/driver.cpp)
target_link_libraries(driver ${catkin_LIBRARIES} MIP_SDK)

add_executable(imu_node src/imu_node.cpp src/3dmgx3.cpp)
target_link_libraries(imu_node ${catkin_LIBRARIES} MIP_SDK)



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
