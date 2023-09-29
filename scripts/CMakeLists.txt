file(GLOB PYTHON_SCRIPTS "*.py")
foreach(PYTHON_SCRIPT ${PYTHON_SCRIPTS})
    get_filename_component(FILE_NAME ${PYTHON_SCRIPT} NAME)
    configure_file(${PYTHON_SCRIPT} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${FILE_NAME} COPYONLY)
endforeach()

configure_file(bash/build_and_run.sh.in ${CMAKE_SOURCE_DIR}/build_and_run.sh @ONLY)
configure_file(bash/env.sh.in ${CMAKE_SOURCE_DIR}/.env.sh @ONLY)
configure_file(bash/grade.sh.in ${CMAKE_SOURCE_DIR}/grade.sh @ONLY)