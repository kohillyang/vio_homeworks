add_executable(testCurveFitting CurveFitting.cpp)
target_link_libraries(testCurveFitting ${PROJECT_NAME}_backend)

add_executable(testCurveFitting2 CurveFitting2.cpp)
target_link_libraries(testCurveFitting2 ${PROJECT_NAME}_backend)