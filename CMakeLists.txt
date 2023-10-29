cmake_minimum_required(VERSION 3.8)

project(amp-tools)
#set(CMAKE_BUILD_TYPE Debug)
if(DEFINED CMAKE_BUILD_TYPE)
    message("Building type is DEBUG")
else()
    set(CMAKE_BUILD_TYPE Release)
    message("Building type is RELEASE")
endif()

# Flag for determining if the library should be built
if(NOT DEFINED AMP_BUILD_LIB)
    option(AMP_BUILD_LIB "Build library (ON), link only (OFF)" OFF)
endif()

if(NOT DEFINED AMP_EXCLUDE_VIS)
    option(AMP_EXCLUDE_VIS "Build without the visualizer" OFF)
endif()

if(NOT DEFINED AMP_EXCLUDE_LOGS)
    option(AMP_EXCLUDE_LOGS "Build without logging" OFF)
endif()

# C++ libs
find_package (Eigen3 3.3.4 REQUIRED NO_MODULE)
find_package(OpenSSL REQUIRED)
if(AMP_BUILD_LIB)
    find_package(Boost 1.70 REQUIRED)
endif()

# Python stuff
find_package(PythonInterp)
find_package(PythonLibs 3.4 REQUIRED)
message("Python Executable: ${Python_EXECUTABLE}")
message("Python Include Directories: ${PythonLibs_INCLUDE_DIRS}")
message("Python Libraries: ${PythonLibs_LIBRARIES}")
set(AMP_PYTHONPATH "" CACHE STRING "Manually specify the PYTHONPATH")

# Directory for input and output files
set(AMP_FILE_IO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/file_dump)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message("Detected Linux operating system")
    message("Detected Linux architecture: ${SYSTEM_ARCHITECTURE}")

    execute_process(COMMAND bash -c "lsb_release -rs"
        OUTPUT_VARIABLE LINUX_UBUNTU_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(LINUX_UBUNTU_VERSION VERSION_EQUAL "22.04")
        message("Detected Linux OS version: 22.04")
        set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/ubuntu22)
    elseif(LINUX_UBUNTU_VERSION VERSION_EQUAL "20.04")
        message("Detected Linux OS version: 20.04")
        set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/ubuntu20)
    else()
        message(WARNING "Unsupported Linux operating system: ${LINUX_UBUNTU_VERSION}, defaulting to ubuntu20 build")
        set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/ubuntu20)
    endif()
    set(OS_LD_LIBRARY_PATH_VAR "LD_LIBRARY_PATH")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    message("Detected macOS operating system")
    message("Detected macOS architecture: ${SYSTEM_ARCHITECTURE}")
	message("Detected macOS system processor: ${CMAKE_SYSTEM_PROCESSOR}")
	
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        message("Detected macOS x86 architecture version: ${CMAKE_SYSTEM_VERSION}")
        set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/macos_x86)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")    
        message("Detected macOS arm64 (Apple Silicon) architecture version: ${CMAKE_SYSTEM_VERSION}")
        set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/macos_arm64)
    else()
        message(FATAL_ERROR "Unsupported macos architecture: ${SYSTEM_ARCHITECTURE}, defaulting to macos_x86")
    endif()
    set(OS_LD_LIBRARY_PATH_VAR "DYLD_LIBRARY_PATH")
else()
    message(FATAL_ERROR "Unsupported operating system: ${CMAKE_SYSTEM_NAME}")
endif()

link_directories(${CMAKE_INSTALL_PREFIX}/lib)
set(AMP_OS_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(AMP_EXECUTABLE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/bin)
set(AMP_EXECUTABLE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})


set(AMP_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/include
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

    # Use Grapefruit as header-only
    list(APPEND AMP_DEPENDENCY_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/grapefruit/grapefruit)
    message("======== DEPENDENCY INCLUDE DIRS: ${AMP_DEPENDENCY_INCLUDE_DIRS}")

    # Native
    add_subdirectory(include/private)
    add_subdirectory(src/private)
    add_subdirectory(executables)
else()

endif()

add_subdirectory(scripts)
add_subdirectory(include/public)
add_subdirectory(src/public)
add_subdirectory(ws)

# Copy the executables out of the install directory
file(COPY ${AMP_EXECUTABLE_DIRECTORY} DESTINATION ${AMP_EXECUTABLE_OUTPUT_DIRECTORY})
