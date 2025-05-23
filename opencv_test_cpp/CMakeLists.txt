cmake_minimum_required(VERSION 3.10)
project(opencv_test)

set(CMAKE_CXX_STANDARD 23)

# 指定 OpenCV 安装目录（根据你自己的路径来）
set(OpenCV_DIR "C:\\Program Files_Coding\\tools\\opencv\\build\\x64\\vc16\\lib")

# 查找 OpenCV 包
find_package(OpenCV REQUIRED)

# 包含头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})

# 编译主程序
add_executable(opencv_test main.cpp)

# 链接 OpenCV 库
target_link_libraries(opencv_test ${OpenCV_LIBS})
