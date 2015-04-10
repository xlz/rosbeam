cmake_minimum_required(VERSION 2.8.3)
project(rosbeam C CXX)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED COMPONENTS roscpp rostime rosconsole roscpp_serialization)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime std_msgs nav_msgs geometry_msgs tf2_msgs)

set(T "bridge")
add_executable(${T}
  src/bridge.cc
)
target_link_libraries(${T} ${catkin_LIBRARIES} rt)
set_target_properties(${T} PROPERTIES LINK_FLAGS "-static-libstdc++ -static-libgcc")
add_custom_command(TARGET ${T} POST_BUILD COMMAND ${CMAKE_STRIP} $<TARGET_FILE:${T}>)
install(TARGETS ${T} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(T "texclient-inject")
add_library(${T} SHARED
  src/texclient-inject.c
)
set_target_properties(${T} PROPERTIES COMPILE_FLAGS "-fno-lto -fvisibility=hidden" LINK_FLAGS "-fno-lto -Wl,--version-script -Wl,${CMAKE_SOURCE_DIR}/versions.txt")
target_link_libraries(${T} dl rt ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS ${T} LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
