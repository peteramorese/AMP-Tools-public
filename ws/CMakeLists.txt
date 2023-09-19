file(GLOB USER_SOURCES *.cpp)
add_executable(main ${USER_SOURCES})
target_link_libraries(main PRIVATE
    yaml-cpp
	Eigen3::Eigen
    AMPCorePrivate
    AMPCorePublic
)
target_include_directories(main PUBLIC
    ${AMP_INCLUDE_DIRS}
    ${AMP_DEPENDENCY_INCLUDE_DIRS}
)

if(AMP_EXCLUDE_VIS)
    target_compile_definitions(main PRIVATE AMP_EXCLUDE_VIS)
endif()

if (AMP_EXCLUDE_LOGS)
    message("Building main without logs")
    target_compile_definitions(main PRIVATE AMP_EXCLUDE_LOGS)
endif()

