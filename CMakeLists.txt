cmake_minimum_required(VERSION 3.10)
project(camera_bridge)
cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 基础依赖 ----
set(OpenCV_DIR "/home/pi/thirdparty/opencv_build")
set(TensorRT_ROOT "/usr")
set(CUDA_ROOT /usr/local/cuda)
set(realsense2_DIR "/usr/lib/aarch64-linux-gnu/cmake/realsense2")
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)
find_package(k4a REQUIRED)
find_package(CUDA REQUIRED)


if (NOT TARGET CUDA::cudart)
    add_library(CUDA::cudart INTERFACE IMPORTED)
    set_target_properties(CUDA::cudart PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CUDA_TOOLKIT_ROOT_DIR}/include"
        INTERFACE_LINK_LIBRARIES "${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcudart.so"
    )
endif()

# ---- 检查ROS环境 ----
if(DEFINED ENV{ROS_DISTRO} OR DEFINED CATKIN_DEVEL_PREFIX)
    message(STATUS "🔹 Detected ROS, building with ROS support")
    set(BUILD_WITH_ROS ON)

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        serial
    )

    catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport serial
    )
else()
    message(STATUS "🔸 No ROS detected, building without ROS")
    set(BUILD_WITH_ROS OFF)
endif()

include_directories(
    /home/pi/thirdparty/opencv_build/opencv/build/lib
    ${catkin_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${TensorRT_ROOT}/include/aarch64-linux-gnu
    inc
)

if(BUILD_WITH_ROS)
    include_directories(${catkin_INCLUDE_DIRS})
endif()


# 链接路径 
link_directories(
    ${TensorRT_ROOT}/lib/aarch64-linux-gnu
    ${CUDA_ROOT}/targets/aarch64-linux/lib
    ${OpenCV_LIBRARY_DIRS}
    /usr/local/lib
)

# YOLO子模块
add_subdirectory(src/yolo)


#通用可执行文件
set(COMMON_LIBS
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    nvinfer
    nvonnxparser
    CUDA::cudart
    yolo
    -lffi
)
# #---realsense---
# add_executable(rs_viewer src/rs_detect.cpp src/realsense.cpp src/myinfer.cpp src/yolo/yolo.cu)
# target_link_libraries(rs_viewer PRIVATE

#     realsense2::realsense2
#     ${COMMON_LIBS}
# )


# #---K4A---
# add_executable(k4a_detect src/k4a_detect.cpp src/camera_k4a.cpp src/myinfer.cpp src/yolo/yolo.cu)
# target_link_libraries(k4a_detect PRIVATE

#     k4a::k4a
#     ${COMMON_LIBS}
# )

# #---one_eye---
# add_executable(one_eye src/one_eye.cpp src/myinfer.cpp src/yolo/yolo.cu src/realsense.cpp)
# target_link_libraries(one_eye PRIVATE

#     realsense2::realsense2
#     ${COMMON_LIBS}
# )
# #---dynamic_rocog---
# add_executable(dynamic_recog src/dynamic_recog.cpp src/grid_state.cpp src/myinfer.cpp src/yolo/yolo.cu src/camera_k4a.cpp)
# target_link_libraries(dynamic_recog PRIVATE

#     k4a::k4a
#     ${COMMON_LIBS}
# )

#             ROS可执行文件（仅在ROS下编译）
if(BUILD_WITH_ROS)
    #---k4a_detect_ros---
    add_executable(k4a_detect_ros src/k4a_detect.cpp src/camera_k4a.cpp src/myinfer.cpp src/yolo/yolo.cu)
    target_compile_definitions(k4a_detect_ros PRIVATE BUILD_WITH_ROS)
    target_include_directories(k4a_detect_ros PRIVATE
        ${catkin_INCLUDE_DIRS}
        inc
    )
    target_link_libraries(k4a_detect_ros PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        nvinfer
        nvonnxparser
        k4a::k4a
        CUDA::cudart
        yolo
        pthread
        -lffi
    )

    add_executable(k4a_serial_node src/serial.cpp)
    target_link_libraries(k4a_serial_node
        ${catkin_LIBRARIES}
    )

    # #---rs_viewer_ros---
    # add_executable(rs_detect_ros src/rs_detect.cpp src/realsense.cpp src/myinfer.cpp src/yolo/yolo.cu)
    # target_compile_definitions(rs_detect_ros PRIVATE BUILD_WITH_ROS)
    # target_include_directories(rs_detect_ros PRIVATE ${catkin_INCLUDE_DIRS} inc)
    # target_link_libraries(rs_detect_ros PRIVATE
    #     ${catkin_LIBRARIES}
    #     ${OpenCV_LIBS}
    #     ${PCL_LIBRARIES}
    #     nvinfer
    #     nvonnxparser
    #     realsense2::realsense2
    #     CUDA::cudart
    #     yolo
    #     pthread
    #     -lffi
    # )

    # #---one_eye_ros---
    # add_executable(one_eye_ros src/one_eye.cpp src/myinfer.cpp src/yolo/yolo.cu src/realsense.cpp)
    # target_compile_definitions(one_eye_ros PRIVATE BUILD_WITH_ROS)
    # target_include_directories(one_eye_ros PRIVATE ${catkin_INCLUDE_DIRS} inc)
    # target_link_libraries(one_eye_ros PRIVATE
    #     ${catkin_LIBRARIES}
    #     ${OpenCV_LIBS}
    #     ${PCL_LIBRARIES}
    #     nvinfer
    #     nvonnxparser
    #     realsense2::realsense2
    #     CUDA::cudart
    #     yolo
    #     pthread
    #     -lffi
    # )
   
    #---dynamic_recog---#
    # add_executable(dynamic_recog_ros src/dynamic_recog.cpp src/grid_state.cpp src/camera_k4a.cpp src/myinfer.cpp src/yolo/yolo.cu src/ros_interface.cpp)
    # target_include_directories(dynamic_recog_ros PRIVATE
    # target_compile_definitions (dynamic_recog_ros PRIVATE BUILD_WITH_ROS)
    #     ${catkin_INCLUDE_DIRS}
    #     inc
    # )
    # target_link_libraries(dynamic_recog_ros PRIVATE
    #     ${catkin_LIBRARIES}
    #     ${OpenCV_LIBS}
    #     ${PCL_LIBRARIES}
    #     nvinfer
    #     nvonnxparser
    #     k4a::k4a
    #     CUDA::cudart
    #     yolo
    #     pthread
    #     -lffi
    # )



endif()
