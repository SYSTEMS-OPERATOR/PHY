#!/usr/bin/env bash
# Minimal Yocto build wrapper
set -e

BUILD_DIR=${BUILD_DIR:-build-yocto}
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
echo "[yocto] Building image..."
# Placeholder command
sleep 1

echo "[yocto] Image ready"
