# Experimental muscle validation

These scripts overlay MyoFullBody moment-arm curves with experimental data
ported from `musclemimic_dev` PR #32.

Run from the repository root:

```bash
uv run --extra dev python scripts/experimental_validation/compare_sim_exp_elbow.py
uv run --extra dev python scripts/experimental_validation/compare_sim_exp_lower.py
uv run --extra dev python scripts/experimental_validation/compare_sim_exp_shoulder.py
```

Plots are written beneath `scripts/output/experimental_validation/`.

Source datasets are kept in `data/`:

- `pigeon_datasets.csv`: elbow moment-arm data used by the Pigeon comparison.
- `S3_Datasets.mat`: lower-limb experimental moment-arm collection.
- `shoulder_dataset_Ackland.csv`: tidy shoulder abduction moment-arm data.

The scripts resolve model and data paths relative to this repository, can be
run from any working directory, and never modify the source MJCF files.
