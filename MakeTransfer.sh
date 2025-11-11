#!/bin/bash
set -e  # Exit immediately if any command fails

BUILD_DIR="build"

if [ "$1" == "clean" ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
    exit 0
fi

# Create the build directory if it doesn't exist
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "Configuring project..."
cmake .. -DCMAKE_BUILD_TYPE=Release

echo "Building TransferGame..."
cmake --build . --config Release

cd ..
echo "Build complete! Executable is in build/ as TransferGame"
