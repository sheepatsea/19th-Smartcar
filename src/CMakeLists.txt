cmake_minimum_required(VERSION 3.4...3.18)
project(patrol_car)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -mcpu=native -flto -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(INCLUDE_PATH "/usr/local/include")
set(LIB_PATH "/usr/local/lib")
#---------------------------------------------------------------------
#       [ Include、Lib  : Define ] ==> []
#---------------------------------------------------------------------

set(COMMON_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/")

set(COMMON_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include"
                        "${PROJECT_SOURCE_DIR}/config"
                        "${PROJECT_SOURCE_DIR}/src"
                        "${PROJECT_SOURCE_DIR}/tool")


link_directories(${COMMON_LIB_DIR})
include_directories( ${COMMON_INCLUDE_DIR} )
#---------------------------------------------------------------------
#       [ Include、Lib  : Path ] ==> [ glib opencv ]
#---------------------------------------------------------------------
find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )    
include_directories( ${OpenCV_INCLUDE_DIRS} )

pkg_search_module(SERIAL REQUIRED libserial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

# find ppnc
pkg_search_module(PPNC REQUIRED ppnc)
include_directories(${PPNC_INCLUDE_DIRS})
link_directories(${PPNC_LIBRARY_DIRS})

# find onnx
pkg_search_module(ONNX REQUIRED onnx)
include_directories(${ONNX_INCLUDE_DIRS})
link_directories(${ONNX_LIBRARY_DIRS})
#---------------------------------------------------------------------
#               [ bin ] ==> [ demo ]
#---------------------------------------------------------------------
# 图像采集
set(COL_PROJECT_NAME "collection")
set(COL_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/collection.cpp)
add_executable(${COL_PROJECT_NAME} ${COL_PROJECT_SOURCES})
target_link_libraries(${COL_PROJECT_NAME} pthread )
target_link_libraries(${COL_PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${COL_PROJECT_NAME} ${SERIAL_LIBRARIES})

# 图像合成
set(IMG2V_PROJECT_NAME "img2video")
set(IMG2V_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/img2video.cpp)
add_executable(${IMG2V_PROJECT_NAME} ${IMG2V_PROJECT_SOURCES})
target_link_libraries(${IMG2V_PROJECT_NAME} pthread )
target_link_libraries(${IMG2V_PROJECT_NAME} ${OpenCV_LIBS})

# 相机标定
set(CAL_PROJECT_NAME "calibration")
set(CAL_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/calibration.cpp)
add_executable(${CAL_PROJECT_NAME} ${CAL_PROJECT_SOURCES})
target_link_libraries(${CAL_PROJECT_NAME} pthread )
target_link_libraries(${CAL_PROJECT_NAME} ${OpenCV_LIBS})

# 相机测试
set(CAM_PROJECT_NAME "camera")
set(CAM_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tool/camera.cpp)
add_executable(${CAM_PROJECT_NAME} ${CAM_PROJECT_SOURCES})
target_link_libraries(${CAM_PROJECT_NAME} pthread )
target_link_libraries(${CAM_PROJECT_NAME} ${OpenCV_LIBS})
#---------------------------------------------------------------------
#               [ bin ] ==> [ main ]
#---------------------------------------------------------------------
set(ICAR_PROJECT_NAME "icar")
set(ICAR_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/src/icar.cpp)
add_executable(${ICAR_PROJECT_NAME} ${ICAR_PROJECT_SOURCES})
target_link_libraries(${ICAR_PROJECT_NAME} ${PPNC_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME} ${ONNX_LIBRARIES})
target_link_libraries(${ICAR_PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${ICAR_PROJECT_NAME} pthread )
target_link_libraries(${ICAR_PROJECT_NAME} ${SERIAL_LIBRARIES})



