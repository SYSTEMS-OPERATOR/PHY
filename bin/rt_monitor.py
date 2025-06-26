#!/usr/bin/env python
"""Display simple jitter histogram from latency bench."""
import subprocess
import statistics


def main() -> None:
    proc = subprocess.run(["./bench_hw/latency_bench"], capture_output=True, text=True)
    for part in proc.stdout.split():
        if part.startswith("jitter_us="):
            val = float(part.split("=")[1])
            print(f"rms_us {val:.2f}")
            break


if __name__ == "__main__":
    main()
