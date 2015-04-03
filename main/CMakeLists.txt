cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(MY_GRAND_PROJECT)
find_package(PCL 1.3 REQUIRED)
find_package( OpenCV REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# directories
set (LIBVISO2_SRC_DIR ../libs/viso2)

# include directory
include_directories("${LIBVISO2_SRC_DIR}")

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# sources
FILE(GLOB LIBVISO2_SRC_FILES "../libs/viso2/*.cpp")

add_executable(main main.cpp 
	../classes/PointCloud/PointCloud.cpp 
	../classes/Disparity/Disparity.cpp 
	../classes/Kalman/Kalman.cpp 
	../classes/Kalman/MyKalman.cpp 
	../classes/Vehicle/Vehicle.cpp 
	../classes/Counter/Counter.cpp 
	../classes/Pressentation/Pressent.cpp 
	../classes/ransac/ransac.cpp 
	../classes/odometry/odometry.cpp
	${LIBVISO2_SRC_FILES})
target_link_libraries(main ${PCL_LIBRARIES} ${OpenCV_LIBS})

ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    # Other flags
)