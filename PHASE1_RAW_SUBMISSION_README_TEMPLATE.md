# Phase 1 raw submission — unprocessed

## Mandatory declaration

**UNPROCESSED — NO CALIBRATION OR ACCURACY CLAIM**

This submission contains unmodified Intel RealSense D455 and Livox Mid-360
recordings plus acquisition metadata. No transform was estimated, no
validation recording was used for fitting, and no reprojection percentage was
calculated before this raw submission.

## Submission identity

- experiment ID: `<fill>`
- rig ID: `<fill>`
- acquisition protocol: `phase1-physical-acquisition/v1.0`
- frozen evaluator: `phase1-evaluator-v1.0.0`
- calibration session ID(s): `<fill>`
- validation session ID(s): `<fill>`
- operator(s): `<fill>`
- capture start/end UTC: `<fill>`

## Frozen split declaration

- split manifest: `manifests/split_assignment.csv`
- split-manifest SHA-256: `<fill>`
- protocol SHA-256: `<fill>`
- complete raw inventory/checksum file: `checksums/SHA256SUMS.txt`

Calibration and validation capture IDs, session IDs, paths, poses, and roles
were assigned before processing. Validation recordings remain unavailable to
calibration fitting until the calibration-only transform is frozen and hashed.

## Integrity-gate result

- command: `python scripts/validate_phase1_capture_submission.py <root> --verify-frozen`
- audit report: `<fill path>`
- audit status: `<PASS/FAIL>`
- raw files modified after hashing: `NO`

Any failed or excluded take must be explained using acquisition-QA evidence
known before reprojection error is calculated. A high reprojection error is not
an acceptable exclusion reason.
