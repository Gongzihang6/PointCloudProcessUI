cmake_minimum_required(VERSION 3.16)
project(PointCloudProcessUI LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 1. 设置 Qt6 路径
set(CMAKE_PREFIX_PATH "F:/dev/Qt/6.9.3/msvc2022_64")

# 2. [核心修复] 查找库时，必须加上 OpenGLWidgets
find_package(Qt6 COMPONENTS Widgets Gui Core OpenGLWidgets Network Concurrent REQUIRED)


find_package(OpenCV REQUIRED COMPONENTS core imgproc)
message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


# 3. 包含头文件目录
include_directories(include)

# 4. 收集源文件 (推荐显式列出，防止漏文件，这里先保持你原本的写法)
file(GLOB_RECURSE SOURCES "src/*.cpp" "main.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

# 5. 查找 PCL (保持不变)
find_package(PCL 1.14 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 6. 添加可执行文件 (建议加上 WIN32 防止弹出黑框)
add_executable(${PROJECT_NAME} WIN32 ${SOURCES} ${HEADERS})

# ==========================================
# 解决 Debug 模式下 PCL/Eigen 导致 obj 文件过大的问题
# ==========================================
if(MSVC)
    target_compile_options(PointCloudProcessUI PRIVATE /bigobj)
endif()

# 7. 链接库
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Qt6::Widgets 
    Qt6::Gui 
    Qt6::Core 
    Qt6::OpenGLWidgets  # 现在上面 find_package 找到了，这里才能链接成功
    Qt6::Network
    Qt6::Concurrent
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)


# F:\dev\Qt\6.9.3\msvc2022_64\bin\windeployqt.exe --release --force "F:\Gongzihang\2026\PointCloudProcessUI\build\Release\PointCloudProcessUI.exe"
