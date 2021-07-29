cmake_minimum_required(VERSION 2.8.3)
project(shape_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Define path of darknet folder here.
find_path(DARKNET_PATH
  NAMES "Makefile"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/src/darknet_lib/")
message("Darknet path dir = ${DARKNET_PATH}")
add_definitions(-DDARKNET_FILE_PATH="${DARKNET_PATH}")

# Find CUDA
FIND_PACKAGE(CUDA)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
#    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
  )
  add_definitions(-DGPU)
else()
  list(APPEND LIBRARIES "m")
endif()

# Find X11
MESSAGE ( STATUS "Searching for X11..." )
FIND_PACKAGE ( X11 REQUIRED )
IF ( X11_FOUND )
  INCLUDE_DIRECTORIES ( ${X11_INCLUDE_DIR} )
  LINK_LIBRARIES ( ${X11_LIBRARIES} )
  MESSAGE ( STATUS " X11_INCLUDE_DIR: " ${X11_INCLUDE_DIR} )
  MESSAGE ( STATUS " X11_LIBRARIES: " ${X11_LIBRARIES} )
ENDIF ( X11_FOUND )


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
    rostime
    roscpp
    rosconsole
    roscpp_serialization
    sensor_msgs
    image_transport
    cv_bridge
    actionlib
    prometheus_msgs
    tf2_eigen
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
    thread
    date_time
)
find_package(OpenCV REQUIRED)

# Enable OPENCV in darknet
add_definitions(-DOPENCV)
add_definitions(-O4 -g)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src/ellipse_lib
  src/darknet_lib/src
  src/darknet_lib/include
  src/darknet_ros/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INUCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ../common/include
)


if (CUDA_FOUND)

  link_directories(
    /usr/local/cuda/lib64
  )

  cuda_add_library(darknet_ros_lib
    src/darknet_ros/src/YoloObjectDetector.cpp
    src/darknet_ros/src/image_interface.c

    src/darknet_lib/src/activation_layer.c        src/darknet_lib/src/im2col.c
    src/darknet_lib/src/activations.c             src/darknet_lib/src/image.c
    src/darknet_lib/src/avgpool_layer.c           src/darknet_lib/src/layer.c
    src/darknet_lib/src/batchnorm_layer.c         src/darknet_lib/src/list.c
    src/darknet_lib/src/blas.c                    src/darknet_lib/src/local_layer.c
    src/darknet_lib/src/box.c                     src/darknet_lib/src/lstm_layer.c
    src/darknet_lib/src/col2im.c                  src/darknet_lib/src/matrix.c
    src/darknet_lib/src/connected_layer.c         src/darknet_lib/src/maxpool_layer.c
    src/darknet_lib/src/convolutional_layer.c     src/darknet_lib/src/network.c
    src/darknet_lib/src/cost_layer.c              src/darknet_lib/src/normalization_layer.c
    src/darknet_lib/src/crnn_layer.c              src/darknet_lib/src/option_list.c
    src/darknet_lib/src/crop_layer.c              src/darknet_lib/src/parser.c
    src/darknet_lib/src/cuda.c                    src/darknet_lib/src/region_layer.c
    src/darknet_lib/src/data.c                    src/darknet_lib/src/reorg_layer.c
    src/darknet_lib/src/deconvolutional_layer.c   src/darknet_lib/src/rnn_layer.c
    src/darknet_lib/src/demo.c                    src/darknet_lib/src/route_layer.c
    src/darknet_lib/src/detection_layer.c         src/darknet_lib/src/shortcut_layer.c
    src/darknet_lib/src/dropout_layer.c           src/darknet_lib/src/softmax_layer.c
    src/darknet_lib/src/gemm.c                    src/darknet_lib/src/tree.c
    src/darknet_lib/src/gru_layer.c               src/darknet_lib/src/utils.c
    src/darknet_lib/src/upsample_layer.c          src/darknet_lib/src/logistic_layer.c
    src/darknet_lib/src/l2norm_layer.c            src/darknet_lib/src/yolo_layer.c

    src/darknet_lib/examples/art.c                src/darknet_lib/examples/lsd.c
    src/darknet_lib/examples/attention.c          src/darknet_lib/examples/nightmare.c
    src/darknet_lib/examples/captcha.c            src/darknet_lib/examples/regressor.c
    src/darknet_lib/examples/cifar.c              src/darknet_lib/examples/rnn.c
    src/darknet_lib/examples/classifier.c         src/darknet_lib/examples/segmenter.c
    src/darknet_lib/examples/coco.c               src/darknet_lib/examples/super.c
    src/darknet_lib/examples/darknet.c            src/darknet_lib/examples/tag.c
    src/darknet_lib/examples/detector.c           src/darknet_lib/examples/yolo.c
    src/darknet_lib/examples/go.c

    src/darknet_lib/src/activation_kernels.cu     src/darknet_lib/src/crop_layer_kernels.cu
    src/darknet_lib/src/avgpool_layer_kernels.cu  src/darknet_lib/src/deconvolutional_kernels.cu
    src/darknet_lib/src/blas_kernels.cu           src/darknet_lib/src/dropout_layer_kernels.cu
    src/darknet_lib/src/col2im_kernels.cu         src/darknet_lib/src/im2col_kernels.cu
    src/darknet_lib/src/convolutional_kernels.cu  src/darknet_lib/src/maxpool_layer_kernels.cu
  )

  target_link_libraries(darknet_ros_lib
    m
    pthread
    stdc++
    cuda
    cudart
    cublas
    curand
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )

  cuda_add_executable(darknet_ros
    src/darknet_ros/src/yolo_object_detector_node.cpp
  )

else()

  add_library(darknet_ros_lib
    src/darknet_ros/src/YoloObjectDetector.cpp
    src/darknet_ros/src/image_interface.c

    src/darknet_lib/src/activation_layer.c        src/darknet_lib/src/im2col.c
    src/darknet_lib/src/activations.c             src/darknet_lib/src/image.c
    src/darknet_lib/src/avgpool_layer.c           src/darknet_lib/src/layer.c
    src/darknet_lib/src/batchnorm_layer.c         src/darknet_lib/src/list.c
    src/darknet_lib/src/blas.c                    src/darknet_lib/src/local_layer.c
    src/darknet_lib/src/box.c                     src/darknet_lib/src/lstm_layer.c
    src/darknet_lib/src/col2im.c                  src/darknet_lib/src/matrix.c
    src/darknet_lib/src/connected_layer.c         src/darknet_lib/src/maxpool_layer.c
    src/darknet_lib/src/convolutional_layer.c     src/darknet_lib/src/network.c
    src/darknet_lib/src/cost_layer.c              src/darknet_lib/src/normalization_layer.c
    src/darknet_lib/src/crnn_layer.c              src/darknet_lib/src/option_list.c
    src/darknet_lib/src/crop_layer.c              src/darknet_lib/src/parser.c
    src/darknet_lib/src/cuda.c                    src/darknet_lib/src/region_layer.c
    src/darknet_lib/src/data.c                    src/darknet_lib/src/reorg_layer.c
    src/darknet_lib/src/deconvolutional_layer.c   src/darknet_lib/src/rnn_layer.c
    src/darknet_lib/src/demo.c                    src/darknet_lib/src/route_layer.c
    src/darknet_lib/src/detection_layer.c         src/darknet_lib/src/shortcut_layer.c
    src/darknet_lib/src/dropout_layer.c           src/darknet_lib/src/softmax_layer.c
    src/darknet_lib/src/gemm.c                    src/darknet_lib/src/tree.c
    src/darknet_lib/src/gru_layer.c               src/darknet_lib/src/utils.c
    src/darknet_lib/src/upsample_layer.c          src/darknet_lib/src/logistic_layer.c
    src/darknet_lib/src/l2norm_layer.c            src/darknet_lib/src/yolo_layer.c

    src/darknet_lib/examples/art.c                src/darknet_lib/examples/lsd.c
    src/darknet_lib/examples/attention.c          src/darknet_lib/examples/nightmare.c
    src/darknet_lib/examples/captcha.c            src/darknet_lib/examples/regressor.c
    src/darknet_lib/examples/cifar.c              src/darknet_lib/examples/rnn.c
    src/darknet_lib/examples/classifier.c         src/darknet_lib/examples/segmenter.c
    src/darknet_lib/examples/coco.c               src/darknet_lib/examples/super.c
    src/darknet_lib/examples/darknet.c            src/darknet_lib/examples/tag.c
    src/darknet_lib/examples/detector.c           src/darknet_lib/examples/yolo.c
    src/darknet_lib/examples/go.c
  )

  target_link_libraries(darknet_ros_lib
    m
    pthread
    stdc++
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )

  add_executable(darknet_ros
    src/darknet_ros/src/yolo_object_detector_node.cpp
  )

endif()

target_link_libraries(darknet_ros
  darknet_ros_lib
)

add_dependencies(darknet_ros_lib
  darknet_ros_msgs_generate_messages_cpp
)

## Declare a C++ library

## Libs

add_library(ellipselib
    src/ellipse_lib/spire_ellipsedetector.cpp
    src/ellipse_lib/spire_ellipsedetector.h
)
target_link_libraries(ellipselib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})




add_executable(ellipse_det
    cpp_nodes/ellipse_det.cpp
)
target_link_libraries(ellipse_det ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ellipselib yaml-cpp)

