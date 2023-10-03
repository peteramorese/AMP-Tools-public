file(GLOB HW_SUBDIRECTORIES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/shared")
    message("Found user-created libraries")
    file(GLOB USER_LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/shared/*.cpp")
    add_library(user_library STATIC ${USER_LIB_SOURCES})
    target_link_libraries(user_library PRIVATE
        yaml-cpp
        Eigen3::Eigen
        AMPCorePrivate
        AMPCorePublic
    )
    target_include_directories(user_library PUBLIC
        ${AMP_INCLUDE_DIRS}
        ${AMP_DEPENDENCY_INCLUDE_DIRS}
    )
endif()
foreach(SUBDIR ${HW_SUBDIRECTORIES})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR} AND NOT SUBDIR STREQUAL "shared")
        file(GLOB_RECURSE USER_SOURCES "${SUBDIR}/*.cpp")
        set(TARGET_NAME "main-${SUBDIR}")
        add_executable(${TARGET_NAME} ${USER_SOURCES})
        target_link_libraries(${TARGET_NAME} PRIVATE
            yaml-cpp
            Eigen3::Eigen
            AMPCorePrivate
            AMPCorePublic
        )
        target_include_directories(${TARGET_NAME} PUBLIC
            ${AMP_INCLUDE_DIRS}
            ${AMP_DEPENDENCY_INCLUDE_DIRS}
        )
        if(TARGET user_library)
            target_link_libraries(${TARGET_NAME} PRIVATE user_library)
            target_include_directories(${TARGET_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/shared")
        endif()

        if(AMP_EXCLUDE_VIS)
            target_compile_definitions(${TARGET_NAME} PRIVATE AMP_EXCLUDE_VIS)
        endif()

        if (AMP_EXCLUDE_LOGS)
            message("Building ${TARGET_NAME} without logs")
            target_compile_definitions(${TARGET_NAME} PRIVATE AMP_EXCLUDE_LOGS)
        endif()
    endif()
endforeach()

