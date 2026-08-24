#!/usr/bin/env python3
"""Compatibility launcher for the strict final Phase 1 validation.

The former implementation measured containment inside a dilated checkerboard
polygon. That is useful visualization evidence but is not point-wise
reprojection accuracy, so Phase 1 now uses the disjoint calibration/validation
split, timestamp diagnostics, frame tables, and error-vector validator.
"""

from phase1_final_experiments import main


if __name__ == "__main__":
    main()
