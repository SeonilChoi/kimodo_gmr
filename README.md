# Kimodo & GMR

Text → human motion (Kimodo, SMPL-X) → humanoid robot motion (GMR).

> `develop`: only the generate → retarget → visualize pipeline (robots: `unitree_h1`, `bdx`).
> `main`: every original Kimodo / GMR feature (other robots, BVH/FBX/OptiTrack inputs, demo UI, ...).

## Layout

```
kimodo/                      Kimodo package (text-to-motion, SMPL-X output)
MotionCorrection/            Kimodo foot-skate post-processing extension (motion_correction)
general_motion_retargeting/  GMR package
  params.py                  Robot registry (model XML, IK config, base body, camera distance)
  ik_configs/                SMPL-X → robot IK configs
scripts/
  smplx_to_robot.py          Retarget SMPL-X motion to a robot (.pkl)
  vis_robot_motion.py        Play a retargeted robot motion
assets/
  unitree_h1/, bdx/          Robot MuJoCo models
  body_models/smplx/         SMPL-X body models (not tracked, download separately)
motion_data/, outputs/       Generated data (not tracked)
licenses/                    Kimodo (Apache-2.0) and GMR (MIT) licenses
```

## Install

```bash
conda activate env_kimodo
pip install -e .
```

`MotionCorrection` is built with CMake on install. If a prebuilt
`MotionCorrection/python/motion_correction/_motion_correction*.pyd` already exists, it is reused and no build runs.

## Usage

### 1. Create motion data

```bash
kimodo_gen "A person stands still." --duration 6 --output motion_data\stand
```

Writes `motion_data\stand.npz` and `motion_data\stand_amass.npz`. The `_amass.npz` file is the GMR input.
The model defaults to `Kimodo-SMPLX-RP-v1`.

Tip: run `kimodo_textencoder` in a separate terminal to keep the text encoder loaded between `kimodo_gen` runs.

### 2. Retarget motion data

```bash
python scripts\smplx_to_robot.py --smplx_file motion_data\stand_amass.npz --robot bdx --save_path outputs\bdx_stand.pkl --rate_limit
```

### 3. Visualize motion data

```bash
python scripts\vis_robot_motion.py --robot bdx --robot_motion_path outputs\bdx_stand.pkl
```

`--robot` is `bdx` (default) or `unitree_h1`.

## Adding a robot

1. Put the MuJoCo model under `assets/<robot>/`.
2. Add an SMPL-X IK config at `general_motion_retargeting/ik_configs/smplx_to_<robot>.json`
   (the configs on `main` are good templates).
3. Register the robot in every dict in `general_motion_retargeting/params.py`.
