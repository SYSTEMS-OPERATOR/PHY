#!/bin/bash
# Minimal Yocto image build placeholder
set -e
if [ "$1" == "qemu-test" ]; then
    echo "Running QEMU RT kernel smoke test"
    exit 0
fi

echo "Building Yocto image with PREEMPT_RT"
sleep 1
echo "Image build complete"
