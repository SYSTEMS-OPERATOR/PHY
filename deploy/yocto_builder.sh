#!/usr/bin/env bash
# Simple placeholder for Yocto image build
set -e

IMAGE_DIR=${1:-yocto_build}
mkdir -p "$IMAGE_DIR"
echo "IMAGE=yocto-img" > "$IMAGE_DIR/image.txt"
