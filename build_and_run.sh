#! /bin/bash

export LD_LIBRARY_PATH=/home/peter/code/AMP-Tools/install/ubuntu22/lib:$LD_LIBRARY_PATH
echo $LD_LIBRARY_PATH
mkdir -p build && cd build
cmake ./.. -DAMP_BUILD_LIB=OFF
make
if [ $? == 0 ]; then
    clear
    cd bin
    ./main 
else
    echo "Build Failed!"
fi
