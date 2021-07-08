cmake_minimum_required(VERSION 2.6)
SET(CMAKE_CXX_FLAGS "-std=gnu++17 -O3")

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Link runTests with what we want to test and the GTest and pthread library
add_executable(executeTests sqrt_test.cpp) # executeTests is the linux executable file
add_executable(executeObsGenTests ObsGenerator_test.cpp) # executeTests is the linux executable file
add_executable(executeVerletTests VerletIntegration_test.cpp)
target_link_libraries(executeTests ${GTEST_LIBRARIES} pthread)
target_link_libraries(executeObsGenTests ${GTEST_LIBRARIES} pthread)   
target_link_libraries(executeVerletTests ${GTEST_LIBRARIES} pthread)  