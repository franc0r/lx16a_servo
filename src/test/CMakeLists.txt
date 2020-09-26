cmake_minimum_required(VERSION 3.7)
 

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
 
# Link runTests with what we want to test and the GTest and pthread library
add_executable(test_serial gtest/gtest_serial_cmd.cpp)
target_link_libraries(test_serial
                      PRIVATE GTest::GTest
                      PRIVATE GTest::Main
                      )
                    
add_test(
  NAME test-serial
  COMMAND test_serial
)                      