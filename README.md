# AMP-Tools
### Professor: Morteza Lahijanian
### Developed by: Peter Amorese
Auto-grader, visualization, and other tools for ASEN 5254 (Algorithmic Motion Planning) at CU Boulder.

## Navigation

### Build scripts
Two build scripts are included for your convenience. You can call them using `bash`:
```
bash <script_name>.sh
```
 - `build.sh`: Compile your workspace
 - `build_and_run.sh`: Compile your workspace and run your `main.cpp` executable

After installation, verify the `build_and_run.sh` script works.

### Workspace `ws/`
The workspace direction `ws/` is where you will put all of your code. You will find a `main.cpp` file which contains your `main()` function. Feel free to add any headers `.h` and class definition files `.cpp` into that directory. They will automatically be linked to `main.cpp`.

### File IO `file_dump/`
If you are NOT using C++, this is directory contains an `in` and `out` folder for input and output files respectively.

### Include `include/`
This directory includes all of the header files you need. The files contain documentation that will help you understand the classes and methods. Do not edit these files, this will cause build errors.

### Build files `build/`
Here you will find the build artifacts (compiled binaries, etc.). You will generally not need to worry about files in here. If you would like to manually run your `main` executable, you will find it in `build/bin`. 

### Prebuilt executable tools `bin/`
After you build your project for the first time, this directory will automatically be generated. If you are NOT using C++, you can call these executables the generate workspaces, check solutions, etc. Any file input and output for these executables is done through the `file_dump/` directory. When calling the executable, pass the `-h` (e.g. `./generate_problem -h`) flag to see the command line arguments. 

## Release Setup/Installation
Forking the `https://github.com/peteramorese/AMP-Tools-public` is preferred, especially if you would like to commit your changes to GitHub. If you would not like to fork the repository, you can directly clone the repository.

Currently supported OS:
 - Ubuntu 20.04 (including WSL on Windows)
 - Ubuntu 22.04 (including WSL on Windows)
 - macOS

If your OS is not supported, and the project does not compile, please reach out to me (info at the bottom of the page).

---
### Ubuntu (or WSL)
On Ubuntu (or WSL), first install Eigen 3.3.7 (C++ matrix library), OpenSSL, and Python3.8 headers

```
sudo apt update
sudo apt install libeigen3-dev libssl-dev python3.8-dev
```

To make sure you have a suitable version of Eigen
```
pkg-config --modversion eigen3
```
and verify the output is at least `3.3.7` (this should be the case if you are using Ubuntu 20.04 or newer)

Clone the forked repository anywhere you want:
```
git clone <forked AMP-Tools-public repo url>
cd AMP-Tools-public
```

Update the submodules (yaml-cpp)
```
git submodule update --init --recursive
```

Install the python dependencies
```
cd /path/to/AMP-Tools-public
pip3 install -r requirements.txt 
```

### macOS
Install `homebrew` if you do not already have it
```
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Install Eigen 3.3.7 and OpenSSL. The python headers should already be installed.
```
brew install eigen
brew install openssl

```

Clone the forked repository anywhere you want:
```
git clone <forked AMP-Tools-public repo url>
cd AMP-Tools-public
```

Update the submodules (yaml-cpp)
```
git submodule update --init --recursive
```

Install the python dependencies
```
cd /path/to/AMP-Tools-public
pip3 install -r requirements.txt 
```

## Issues, bugs, comments
Please feel free to raise issues on the GitHub page as they arise. If you experience an issue and are unsure if it is caused by a bug, don't hesitate to reach out to me at `peter.amorese@colorado.edu`.
