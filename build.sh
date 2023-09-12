#! /bin/bash

mkdir -p build && cd build

architecture=$(uname -m)
cmake ./.. -DAMP_BUILD_LIB=OFF -DSYSTEM_ARCHITECTURE="${architecture}" 

make
if [ $? != 0 ]; then
    echo "Build Failed!"
fi
