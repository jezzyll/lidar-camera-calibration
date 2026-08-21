#!/usr/bin/env python3
"""Compatibility launcher for the corrected Phase 1 validation.

The former implementation measured containment inside a dilated checkerboard
polygon. That is useful visualization evidence but is not point-wise
reprojection accuracy, so Phase 1 now always uses the non-circular validator.
"""

from phase1_required_validation import main


if __name__ == "__main__":
    main()
