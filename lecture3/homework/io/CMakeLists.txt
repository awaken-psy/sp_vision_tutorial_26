cmake_minimum_required(VERSION 3.16)
project(io)

# 查找OpenCV
find_package(OpenCV REQUIRED)


# 打印OpenCV信息用于调试
# message(STATUS "OpenCV found: ${OpenCV_FOUND}")
# message(STATUS "OpenCV version: ${OpenCV_VERSION}")
# message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
# message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

# 查找yaml-cpp
find_package(yaml-cpp REQUIRED)

# 创建目标 io
add_library(io STATIC 
    my_camera.cpp
)

# 包含目录 - 明确添加OpenCV包含路径
target_include_directories(io PUBLIC 
    ${OpenCV_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}/opencv2  
    hikrobot/include
)

# 链接目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# 链接库
target_link_libraries(io 
    MvCameraControl 
    ${OpenCV_LIBS}
    yaml-cpp
    usb-1.0
)

# 添加可执行文件
add_executable(example example.cpp)
target_link_libraries(example 
    io 
    ${OpenCV_LIBS}
)