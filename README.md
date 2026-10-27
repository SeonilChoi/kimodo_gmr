# Kimodo & GMR

Text → human motion (Kimodo, SMPL-X) → humanoid robot motion (GMR) in a single repository.

> `main` keeps every original Kimodo / GMR feature.
> `develop` keeps only the generate → retarget → visualize pipeline for Unitree H1 and BDX.

## Layout

```
kimodo/                      Kimodo package (text-to-motion)
MotionCorrection/            Kimodo post-processing C++ extension (motion_correction)
general_motion_retargeting/  GMR package (retargeting / viewer)
scripts/                     GMR entry-point scripts
assets/                      Robot models (+ assets/body_models/smplx, not tracked)
third_party/                 GMR third-party code (poselib)
benchmark/                   Kimodo benchmark tools
docs/                        Original Kimodo / GMR docs
licenses/                    Kimodo (Apache-2.0) and GMR (MIT) licenses
motion_data/, outputs/       Generated data (not tracked)
```

## Install

```bash
conda activate env_kimodo
pip install -e .
```

SMPL-X body models go in `assets/body_models/smplx/` (`SMPLX_NEUTRAL.npz`, ...).

## Create motion data

```bash
kimodo_gen "A person stands still." --model Kimodo-SMPLX-RP-v1 --duration 6 --output motion_data\stand
```

This writes `motion_data\stand.npz` and `motion_data\stand_amass.npz` (the latter is the GMR input).

## Retarget motion data

```bash
python scripts\smplx_to_robot.py --smplx_file motion_data\stand_amass.npz --robot bdx --save_path outputs\bdx_stand.pkl --rate_limit
```

## Visualize motion data

```bash
python scripts\vis_robot_motion.py --robot bdx --robot_motion_path outputs\bdx_stand.pkl
```
