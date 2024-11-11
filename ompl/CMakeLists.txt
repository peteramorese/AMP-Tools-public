project(AMP-OMPL-Example)

cmake_minimum_required (VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)

find_package(ompl REQUIRED) # make sure this is the ompl with the K-CBS implementation
find_package(yaml-cpp REQUIRED)

include_directories(
    includes
    ${OMPL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(${OMPL_LIBRARY_DIRS})

file(GLOB DEMO_SOURCES RELATIVE ${CMAKE_SOURCE_DIR}/src/ src/*.cpp )


include(CMakePrintHelpers)
cmake_print_variables(DEMO_SOURCES)
cmake_print_variables(CMAKE_SOURCE_DIR)

foreach( demoFile ${DEMO_SOURCES} )
    string( REPLACE ".cpp" "" demoName ${demoFile} )
    add_executable( ${demoName} src/${demoFile} )
    # Make sure YourLib is linked to each app
    target_link_libraries( ${demoName} ${OMPL_LIBRARIES} ${YAML_CPP_LIBRARIES} )
endforeach( demoFile ${DEMO_SOURCES} )

# ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")
