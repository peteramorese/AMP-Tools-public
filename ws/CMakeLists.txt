file(GLOB USER_SOURCES *.cpp)
add_executable(main ${USER_SOURCES})
target_link_libraries(main PRIVATE
    yaml-cpp
	Eigen3::Eigen
    AMPCore
	${PYTHON_LIBRARIES}
    #OpenSSL::SSL
    #OpenSSL::Crypto
)
target_include_directories(main PUBLIC
    ${AMP_INCLUDE_DIRS}
    ${AMP_DEPENDENCY_INCLUDE_DIRS}
)
