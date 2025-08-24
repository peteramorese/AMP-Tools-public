# Start from Ubuntu 22.04 (Jammy)
FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies: compiler, CMake, Eigen, OpenSSL, Python, pip, Tkinter, zip
RUN apt-get update && apt-get install -y \
    lsb-release \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libssl-dev \
    python3 \
    python3-pip \
    python3-tk \
    zip \
    && rm -rf /var/lib/apt/lists/*


# Determine Python version and install the corresponding python3.x-dev
# (for Ubuntu 22.04 this is typically Python 3.10)
RUN apt-get update && apt-get install -y python3.10-dev && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /root/AMP-Tools

COPY requirements.txt .

# Install the Python dependencies
RUN pip3 install --no-cache-dir -r requirements.txt
