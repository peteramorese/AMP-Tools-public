#! /bin/bash

mkdir -p build && cd build
cmake ./.. -DAMP_BUILD_LIB=OFF
make
if [ $? != 0 ]; then
    echo "Build Failed!"
fi
