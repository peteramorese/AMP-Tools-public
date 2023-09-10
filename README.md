# AMP-Tools
### Professor: Morteza Lahijanian
### Developed by: Peter Amorese
Auto-grader, visualization, and other tools for ASEN 5254 (Algorithmic Motion Planning) at CU Boulder

## Release Setup/Installation
---
### Ubuntu (or WSL)
On Ubuntu (or WSL), first install Eigen 3.3.7 (C++ matrix library) and OpenSSL

```
sudo apt update
sudo apt install libeigen3-dev libssl-dev
```

To make sure you have a suitable version of Eigen
```
pkg-config --modversion eigen3
```
and verify the output is at least `3.3.7` (this should be the case if you are using Ubuntu 20.04 or newer)

Clone this repository anywhere you want:
```
git clone git@github.com:peteramorese/AMP-Tools.git
```

Update the submodules
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

Install Eigen 3.3.7 and OpenSSL
```
brew install eigen
brew install openssl

```

Clone this repository anywhere you want:
```
git clone git@github.com:peteramorese/AMP-Tools.git
```

Update the submodules
```
git submodule update --init --recursive
```

Install the python dependencies
```
cd /path/to/AMP-Tools-public
pip3 install -r requirements.txt 
```