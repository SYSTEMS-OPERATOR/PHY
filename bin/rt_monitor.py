#!/usr/bin/env python
"""Print simple jitter stats from latency_bench output."""

import subprocess


def main():
    proc = subprocess.run(["./bench_hw/latency_bench"], capture_output=True, text=True)
    print(proc.stdout)


if __name__ == "__main__":
    main()
