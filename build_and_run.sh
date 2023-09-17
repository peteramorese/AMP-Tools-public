#! /bin/bash

while [[ $# -gt 0 ]]; do 
    case "$1" in 
        -r|--rebuild)
            echo "Rebuilding from scratch..."
            rm -rf build            
            shift 1
            ;;
        *)
            echo "Unknown arg '$1'"
            exit 1
            ;;
    esac
done

export LD_LIBRARY_PATH=/Users/nicolasperrault/Desktop/AMP-Tools-public/install/macos_x86/lib:$LD_LIBRARY_PATH
echo $LD_LIBRARY_PATH
mkdir -p build && cd build

architecture=$(uname -m)
cmake ./.. -DAMP_BUILD_LIB=OFF -DSYSTEM_ARCHITECTURE="${architecture}" 

make
if [ $? == 0 ]; then
    clear
    cd bin
    ./main 
else
    echo "Build Failed!"
fi
