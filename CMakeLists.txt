cmake_minimum_required(VERSION 3.8)

project(amp-tools)

find_package (Eigen3 3.3.4 REQUIRED NO_MODULE)
find_package(PythonLibs 3.4 REQUIRED)
find_package(OpenSSL REQUIRED)

if(AMP_BUILD_LIB)
    find_package(Boost 1.70 REQUIRED)
endif()



if(NOT DEFINED AMP_BUILD_LIB)
    option(AMP_BUILD_LIB "Build library (ON), link only (OFF)" OFF)
endif()

set(AMP_FILE_IO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/file_dump)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(AMP_EXECUTABLE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message("Detected Linux operating system")
    message("Detected Linux OS version: ${CMAKE_SYSTEM_VERSION}")
    # Install and Link to static libraries built for the OS
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/linux)
    link_directories(install/linux/lib)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    message("Detected macOS operating system")
    message("Detected macOS version: ${CMAKE_SYSTEM_VERSION}")
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/macos)
    link_directories(install/macos/lib)
else()
    message(FATAL_ERROR "Unsupported operating system: ${CMAKE_SYSTEM_NAME}")
endif()


link_directories(install)


set(AMP_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(AMP_DEPENDENCY_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/yaml-cpp/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

# Core artifacts
if (AMP_BUILD_LIB)
    message("Building AMPCore library")

    # Dependencies
    set(BUILD_SHARED_LIBS ON) # Toggle on for YAML
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/yaml-cpp)
    install(TARGETS yaml-cpp ARCHIVE DESTINATION lib)
    set(BUILD_SHARED_LIBS OFF) # Toggle off for everything else

    # Native
    add_subdirectory(include/private)
    add_subdirectory(src)
    add_subdirectory(executables)
endif()

add_subdirectory(scripts)
add_subdirectory(ws)
