#! /bin/bash

debug_flag=""
while [[ $# -gt 0 ]]; do 
    case "$1" in 
        -r|--rebuild)
            echo "Rebuilding from scratch..."
            rm -rf build            
            rm build_and_run.sh
            rm grade.sh
            shift 1
            ;;
        -d|--debug)
            debug_flag="-DCMAKE_BUILD_TYPE=Debug"
            shift 1
            ;;
        *)
            echo "Unknown arg '$1'"
            exit 1
            ;;
    esac
done

mkdir -p build && cd build

architecture=$(uname -m)
cmake .. -DAMP_BUILD_LIB=OFF -DAMP_EXCLUDE_VIS=OFF -DAMP_EXCLUDE_LOGS=OFF -DSYSTEM_ARCHITECTURE="${architecture}" $debug_flag

make
if [ $? != 0 ]; then
    echo "Build Failed!"
fi
