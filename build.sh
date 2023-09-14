#! /bin/bash

while [[ $# -gt 0 ]]; do 
    case "$1" in 
        -r|--rebuild)
            echo "Rebuilding from scratch..."
            rm -rf build            
            rm build_and_run.sh
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
cmake ./.. -DAMP_BUILD_LIB=OFF -DSYSTEM_ARCHITECTURE="${architecture}" 

make
if [ $? != 0 ]; then
    echo "Build Failed!"
fi
