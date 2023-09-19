configure_file(VisualizeEnvironment.py ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/VisualizeEnvironment.py COPYONLY)
configure_file(bash/build_and_run.sh.in ${CMAKE_SOURCE_DIR}/build_and_run.sh @ONLY)
configure_file(bash/env.sh.in ${CMAKE_SOURCE_DIR}/.env.sh @ONLY)
configure_file(bash/grade.sh.in ${CMAKE_SOURCE_DIR}/grade.sh @ONLY)