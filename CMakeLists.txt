set(project_name lx16a_servo)

project(${project_name})

cmake_minimum_required (VERSION 2.6)

#set compiler flags
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O2 -Wall -std=c++17")
#set Variables
# set(EXECUTABLE_PATH ${PROJECT_SOURCE_DIR}/bin)

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_PATH}/${project_name})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_PATH}/)


find_package(Boost COMPONENTS system REQUIRED)

include_directories(${Boost_INCLUDE_DIR}
                    )

link_directories(${Boost_LIBRARY_DIR} )

#grab files
file(GLOB src_lx16a_servo_lib "src/*.cpp"
             )



#add lib
add_library(lx16a_servo ${src_lx16a_servo_lib})

#add executable
add_executable(test_servo src/Serial.cpp
                          src/test/test.cpp )

#link executable
target_link_libraries(test_servo ${Boost_LIBRARIES}
                                 pthread
                               )


#install
#install(TARGETS template
#        RUNTIME DESTINATION bin
#        )

#install(FILES config/template.xml DESTINATION /etc/template/)
