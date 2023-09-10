file(GLOB CORE_SOURCES *.cpp)

add_library(AMPCorePublic STATIC
    ${CORE_SOURCES}
)

target_link_libraries(AMPCorePublic PUBLIC
    yaml-cpp
    AMPCorePrivate
	${PYTHON_LIBRARIES}
    OpenSSL::SSL
    OpenSSL::Crypto
)

target_include_directories(AMPCorePublic PUBLIC
    ${AMP_INCLUDE_DIRS}
    ${AMP_DEPENDENCY_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR}
    ${Boost_INCLUDE_DIRS}
)
