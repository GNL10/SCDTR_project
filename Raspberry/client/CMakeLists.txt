cmake_minimum_required(VERSION 3.10)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 " )
find_package( Boost COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
add_executable( client client.cpp )
target_link_libraries( client ${Boost_LIBRARIES} pthread )

