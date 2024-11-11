#!/bin/bash

rebuild_flag=false
debug_flag=""

# Process command-line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -r|--rebuild)
            echo "Rebuilding from scratch..."
            rebuild_flag=true
            shift 1
            ;;
        -d|--debug)
            debug_flag="-DCMAKE_BUILD_TYPE=Debug"
            shift 1
            ;;
        *)
            if [ -z "$executable" ]; then
                executable="$1"
            else
                echo "Unknown arg '$1'"
                exit 1
            fi
            shift 1
            ;;
    esac
done

# Rebuild if the -r flag was used
if [ "$rebuild_flag" = true ]; then
    rm -rf CMakeFiles
fi

# Run CMake and Make
mkdir -p build && cd build
cmake $debug_flag ./..
make

if [ $? == 0 ]; then
    cd ..
    if [ -n "$executable" ]; then
        ./build/"$executable" "${@:2}"
        if [ $? -eq 0 ]; then
            python3 visualize.py
        else
            echo "Executable $executable failed with exit code $?."
        fi
    else
        echo "Executable not specified."
    fi
else
    echo "Build Failed!"
fi
