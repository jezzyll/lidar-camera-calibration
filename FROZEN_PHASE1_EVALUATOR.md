# Frozen Phase 1 evaluator

## Frozen identity

- version: `phase1-evaluator-v1.0.0`;
- source commit: `50eb04fb8ed732e4dd8cc01092658bdf8fa08295`;
- freeze date: 28 August 2026;
- state: **FROZEN — do not change the evaluation methodology for the next
  physical experiment**.

The machine-readable source and evidence hashes are in
`FROZEN_PHASE1_EVALUATOR.json`. Run the acquisition-submission validator with
`--verify-frozen` before accepting or processing any new recordings.

The tag and freeze currently exist locally. Publishing the branch/tag to a
remote repository requires separate authorization. The Git tag protects source
history; the JSON hashes additionally protect ignored numeric evidence files.

## Accepted baseline result

| Distance | Returns within 2 px | Evaluated returns | Percentage | Decision |
|---:|---:|---:|---:|:---:|
| 7.25 ft | 1,777 | 1,777 | 100.00% | pass |
| 14.5 ft | 0 | 349 | 0.00% | fail |
| 22.2 ft | 78 | 170 | 45.88% | fail |

Phase 1 remains **OPEN**. The accepted 22.2-ft value is `78/170 = 45.88%`;
only this evidence-backed numerator and denominator are used.

## Freeze rules

1. Do not modify `R`, `t`, `K`, or `D` merely to improve acceptance rates.
2. Do not alter LiDAR-only selection, mask exclusion, threshold definitions,
   synchronization analysis, frame splitting, or aggregation logic.
3. Do not refit the transform on validation captures.
4. Submit and inventory new raw recordings before evaluation.
5. Freeze the new experiment protocol and the bag-level calibration/validation
   assignment before opening validation data in the evaluator.
6. If a genuine method change becomes necessary, create a new version/tag,
   document it, and rerun independent validation. Never overwrite this
   baseline or silently reuse its version name.

Documentation, acquisition manifests, and packaging may be improved without
changing this frozen evaluator, provided the SHA-256 source-file audit remains
unchanged.

`outputs/phase1_final/frozen_calibration.json` is the authoritative transform
record. The legacy `config/calibration.yaml` is not read by the strict evaluator
and must not be cited as the accepted result.
