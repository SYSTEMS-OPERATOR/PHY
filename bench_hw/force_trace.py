"""Analyse torque tracking error."""

import numpy as np


def tracking_error(cmd: np.ndarray, measured: np.ndarray) -> float:
    return float(np.max(np.abs(cmd - measured)))


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("cmd")
    parser.add_argument("measured")
    args = parser.parse_args()
    cmd = np.loadtxt(args.cmd)
    meas = np.loadtxt(args.measured)
    print(tracking_error(cmd, meas))
