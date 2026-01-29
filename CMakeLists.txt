cmake_minimum_required(VERSION 3.14)
project(bytetrack)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

option(WITH_TEST "Build test"  ON)
option(WITH_PYTHON "Build python bindings" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -MMD -Wall -Wextra -Winit-self")
set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

if(WITH_PYTHON)
  set(PYBIND11_ZIP "${CMAKE_CURRENT_SOURCE_DIR}/3rd/pybind11-2.10.4.zip")
  execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar xzf ${PYBIND11_ZIP}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      RESULT_VARIABLE EPYBIND11_UNPACK_RESULT
      OUTPUT_VARIABLE PYBIND11_UNPACK_OUTPUT
      ERROR_VARIABLE PYBIND11_UNPACK_ERROR
  )
endif()

# 1. Look for Eigen3 on the system
find_package(Eigen3 3.3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ByteTrackerImpl.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/KalmanFilter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lapjv.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/STrack.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  # Used when you are compiling the ByteTrack project itself
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  
  # Used by other projects AFTER ByteTrack is installed
  # 'include' here is relative to the CMAKE_INSTALL_PREFIX
  $<INSTALL_INTERFACE:include>
)
  
# This automatically handles include paths, so you don't need the 
# ${CMAKE_CURRENT_BINARY_DIR}/eigen-3.3.9 line anymore.
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

if(WITH_PYTHON)
  set(PYTHON_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/python/pybytetrack.cpp)
  add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/pybind11-2.10.4)
  pybind11_add_module(py${PROJECT_NAME} ${PYTHON_SRC})
  target_link_libraries(py${PROJECT_NAME} PRIVATE ${PROJECT_NAME})
endif()

if(WITH_TEST) 
  add_executable(test_bytetrack ${CMAKE_CURRENT_SOURCE_DIR}/test/test_bytetrack.cpp)
  target_link_libraries(test_bytetrack ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/ByteTracker.h 
        DESTINATION include/)
        
# --- Configuration for find_package(bytetrack) ---
include(CMakePackageConfigHelpers)

# 1. Generate the "Targets" file 
# This tells other projects exactly where the .so file is located after installation
install(EXPORT ByteTrackTargets
    FILE ByteTrackTargets.cmake
    NAMESPACE ByteTrack::
    DESTINATION lib/cmake/ByteTrack
)

# 2. Update the existing install(TARGETS) to associate it with the Export set
# We modify the existing install rule by adding 'EXPORT ByteTrackTargets'
install(TARGETS ${PROJECT_NAME}
    EXPORT ByteTrackTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# 3. Create the Config file
# This is the entry point for find_package
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/ByteTrackConfig.cmake"
"include(CMakePackageConfigHelpers)
include(\"\${CMAKE_CURRENT_LIST_DIR}/ByteTrackTargets.cmake\")
")

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ByteTrackConfig.cmake"
    DESTINATION lib/cmake/ByteTrack
)        
            
        
           
