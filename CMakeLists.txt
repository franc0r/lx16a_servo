set(project_name lx16a_servo)

project(${project_name})

cmake_minimum_required (VERSION 3.10)

enable_testing()


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
# file(GLOB src_lx16a_servo_lib "src/*.cpp"
#                               # "src/francor/*.cpp"
#              )



#add lib
# add_library(lx16a_servo ${src_lx16a_servo_lib})

#add executable
add_executable(test_servo   src/test/test.cpp )

add_executable(test_handler src/test/test_handler.cpp)

#tools
add_executable(find_servos src/tools/find_servos.cpp)
add_executable(servo_set_id src/tools/servo_set_id.cpp)

#link executable
target_link_libraries(test_servo ${Boost_LIBRARIES}
                                 pthread
                                 tinyxml2
                               )

target_link_libraries(test_handler ${Boost_LIBRARIES}
                                 pthread
                                 tinyxml2
                               )

target_link_libraries(find_servos ${Boost_LIBRARIES}
                               pthread
                               tinyxml2
                             )

target_link_libraries(servo_set_id ${Boost_LIBRARIES}
                             pthread
                             tinyxml2
                           )
#install
#install(TARGETS template
#        RUNTIME DESTINATION bin
#        )

#install(FILES config/template.xml DESTINATION /etc/template/)

# add_subdirectory(src/test)