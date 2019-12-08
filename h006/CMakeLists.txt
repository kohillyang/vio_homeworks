project(visio_demo)
cmake_minimum_required(VERSION 2.8.0)
set(CMAKE_CXX_STANDARD 11)
# Check if CXX flags have been set to c++11 -> Setup Eclipse Indexer correctly!
# Also setup the project slightly different
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4" )
    SET(CXX_ENABLED 0)
    LIST(LENGTH CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS LIST_LEN)
    if ( ${LIST_LEN} GREATER 0 )
        SET(CXX_ENABLED 1)
    endif()

    SET(C_ENABLED 0)
    LIST(LENGTH CMAKE_EXTRA_GENERATOR_C_SYSTEM_DEFINED_MACROS LIST_LEN)
    if ( ${LIST_LEN} GREATER 0)
        SET(C_ENABLED 1)
    endif()

    if (${C_ENABLED} EQUAL 1 AND ${CXX_ENABLED} EQUAL 1)
        # Combined project (C and CXX). This will confuse the indexer. For that reason
        # we unsert set the __cplusplus variable for the indexer, assuming C++ is only
        # used for unit tests!
        list(FIND CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS "__cplusplus" GEN_MACRO_INDEX)
        if( ${GEN_MACRO_INDEX} GREATER -1 )
            list(REMOVE_AT CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS ${GEN_MACRO_INDEX})
            list(REMOVE_AT CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS ${GEN_MACRO_INDEX})
        endif()
        SET(CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS ${CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS} CACHE INTERNAL "")
    elseif ( (${CXX_ENABLED} EQUAL 1) AND (${CMAKE_CXX_FLAGS} MATCHES ".*-std=c\\+\\+11.*"))
        add_definitions (-D__cplusplus=201103L)
        # CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS
        list(FIND CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS "199711L" GEN_MACRO_INDEX)
        if( ${GEN_MACRO_INDEX} GREATER -1 )
            list(REMOVE_AT CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS ${GEN_MACRO_INDEX})
            list(INSERT CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS ${GEN_MACRO_INDEX} "201103L")
            SET(CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS ${CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_DEFINED_MACROS} CACHE INTERNAL "")
        endif()
    endif()
endif()
add_definitions("-DENABLE_SSE")
#add_definitions("-D__cplusplus=201402L")
set(CMAKE_CXX_FLAGS "${SSE_FLAGS} -msse4")
#set(CMAKE_EXE_LINKER_FLAGS -fuse-ld=gold)
set(CUSTOM_Qt_ROOT /data3/zyx/yks/qt-everywhere-src-5.13.1/install)
list(APPEND CMAKE_PREFIX_PATH ${CUSTOM_Qt_ROOT}/lib/cmake/Qt5)
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/thirdparty/eigen-eigen-323c052e1731/build)
#list(APPEND CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/thirdparty/pcl/build)


SET(QT_QMAKE_EXECUTABLE ${CUSTOM_Qt_ROOT}/bin/qmake)
#set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /data3/zyx/yks/sources/opencv/build)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /data3/zyx/yks/image_stitcher/thirdparty/VTK/build)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
message(STATUS ${OPENGL_LIBRARIES})
find_package(OpenCV 4.1.0 REQUIRED COMPONENTS core highgui imgproc calib3d video)
find_package(Qt5 COMPONENTS Core Gui Widgets Test Xml OpenGL Network WebSockets REQUIRED)
#FIND_PACKAGE(QGLViewer REQUIRED)
find_package(OpenGL)
find_package(BLAS)
find_package(LAPACK)
find_package(Eigen3 REQUIRED)
find_package( OpenMP REQUIRED)
find_package(VTK REQUIRED)

if(OPENMP_FOUND)
    message(STATUS "OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS}${OpenMP_EXE_LINKER_FLAGS}")
endif()

project(moving_dlt)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Test Qt5::Xml Qt5::OpenGL Qt5::WebSockets
        ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} dl
        ${CSPARSE_LIBRARY}
        ${CHOLMOD_LIBRARIES}
        ${g2o_LIBRARIES}
        ${VTK_LIBRARIES}
        )
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/src/lib/)
