cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)

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

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)


message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)





add_executable(run_euroc 
test/run_euroc.cpp
#        test/CurveFitting.cpp
    src/System.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/feature_tracker.cpp

    src/utility/utility.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp

    src/backend/vertex.cc
    src/backend/edge.cc
    src/backend/problem.cc
    src/backend/vertex_pose.cc
    src/backend/edge_reprojection.cc
    src/backend/edge_imu.cc
    src/backend/edge_prior.cpp
    src/backend/loss_function.cc
    src/backend/imu_integration.cc	
src/camera_models/chessboard/Chessboard.cc
    src/camera_models/calib/CameraCalibration.cc
    src/camera_models/camera_models/Camera.cc
    src/camera_models/camera_models/CameraFactory.cc
    src/camera_models/camera_models/CostFunctionFactory.cc
    src/camera_models/camera_models/PinholeCamera.cc
    src/camera_models/camera_models/CataCamera.cc
    src/camera_models/camera_models/EquidistantCamera.cc
    src/camera_models/camera_models/ScaramuzzaCamera.cc
    src/camera_models/sparse_graph/Transform.cc
    src/camera_models/gpl/gpl.cc
    src/camera_models/gpl/EigenQuaternionParameterization.cc	
)
target_link_libraries(run_euroc 
  -lpthread
        ${OpenCV_LIBS}  
  ${CERES_LIBRARIES}
	${Pangolin_LIBRARIES}
	${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}
  
  ) 



