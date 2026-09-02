# Complex dataset reproducibility

All 120 PLY files can be regenerated without the original chat. The generator uses the real prototype archive and the complete scene specification stored inside this package.

```bash
python code/generate_dataset.py --root . --out regenerated_clouds
```

A verification run for this handoff regenerated all 120 files and compared SHA-256 hashes against the packaged PLY files:

```text
reference PLY: 120
regenerated PLY: 120
missing: 0
extra: 0
SHA-256 mismatches: 0
PASS
```

Machine-readable result: `results/complex_reproducibility.json`.

The complex temporal definitions are also preserved in `assets/scene_spec.csv` and their human-readable meaning is documented in `docs/COMPLEX_DATASET_V4.md` and `labels/complex_sequence_catalog.csv`.
