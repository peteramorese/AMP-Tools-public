#! /bin/bash

git pull
#! /bin/bash

# Make sure executable tag is passed
if [[ $# == 0 ]]; then
    cd ws
    options=$(find . -maxdepth 1 -mindepth 1 -type d)
    echo "Must pass executable tag (bash build_and_run.sh <tag>). Here are the your tag options:"
    while IFS= read -r dir; do
        dir_name=$(basename "$dir")
        echo " - $dir_name"
    done <<< "$options"
    exit 1
fi

executable="main-$1"

shift

#use_valgrind=false

for arg in "$@"; do
    case "$arg" in 
        -r|--rebuild)
            echo "Rebuilding from scratch..."
            rm -rf build            
            rm grade.sh
            rm .env.sh
            shift 1
            ;;
        #-d|--valgrind)
        #    echo "Running valgrind on your executable"
        #    if command -v valgrind >/dev/null 2>&1; then
        #        use_valgrind=true
        #    else
        #        echo "Valgrind not installed, cannot use --valgrind (-d) debug flag"
        #    fi
        #    shift 1
        #    ;;
        *)
            echo "Unknown arg '$arg'"
            exit 1
            ;;
    esac
done

export DYLD_LIBRARY_PATH="/Users/yusif/Documents/Coding_Projects/Bugs/AMP-Tools-public/install/macos_x86/lib:$DYLD_LIBRARY_PATH"
echo "Library path: $DYLD_LIBRARY_PATH"
mkdir -p build && cd build

architecture=$(uname -m)
cmake ./.. -DAMP_BUILD_LIB=OFF -DAMP_EXCLUDE_VIS=OFF -DAMP_EXCLUDE_LOGS=OFF -DSYSTEM_ARCHITECTURE="${architecture}" 

make
if [ $? == 0 ]; then
    #clear
    cd bin
    #if [ "use_valgrind" = true ]; then
    #    valgrind ./"$executable"
    #else
    ./"$executable"
    #fi
else
    echo "Build Failed!"
fi
