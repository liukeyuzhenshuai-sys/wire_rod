# Implementation Status V0.3 — Complex Stress Dataset

## Implemented

- Sequential Web inspector with `下一帧 →`, default port **9999**.
- Persistent TrackManager: after frame 0, subsequent frames use the algorithm's committed state rather than GT bootstrap.
- Original V3 simple/unit regression data retained.
- Added **C01-C10, 52 complex temporal frames**; package total is **120 flat-root PLY files**.
- Complex cases combine NEW yaw, abnormal/large gap, non-uniform motion of all historical coils, X/Y/Z drift, rolling observation mismatch, partial/complete occlusion, multi-NEW, layer-2 loading, robot hint error, rebound/non-monotonic motion and nearest-neighbor traps.
- Added semantic rank-lock safety rule: if an entire layer was visible, current count is unchanged, and there is no NEW hint, same-layer rank/order is treated as identity when all rank pairs pass physical gates and collective translation is coherent. This prevents full-chain ID shift when the whole cluster translates by more than half a coil spacing.

## Regression

- Complex continuous regression: **C01-C10 / 52 frames / WRONG_ID=0**.
- Recommended pairwise regression across simple + complex pairs: **87 pairs / WRONG_ID=0 / validator failures=0**.
- NEW: TP=39, FP=0, FN=0 in pairwise regression.
- OCCLUDED: TP=8, FP=0, FN=0 in pairwise regression.
- One conservative UNCERTAIN remains in pairwise regression; this is allowed by product policy.
- 128,769-point S13 geometry benchmark is below the 2 s target in the current Python implementation.

## Important limitation

This is still synthetic stress testing based on seven real coil prototypes. Passing C01-C10 does not prove production correctness. The next validation stage must use real temporal captures and calibrate motion ROI, geometry uncertainty, NEW target error and gap thresholds.

## Re-run

```bash
python tests/run_complex_regression.py
python tests/run_regression.py
python tests/benchmark_s13.py
```
