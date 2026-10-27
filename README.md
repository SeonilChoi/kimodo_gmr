# Kimodo & GMR

Text → human motion (Kimodo, SMPL-X) → humanoid robot motion (GMR).

> `develop`: only the generate (+ `kimodo_demo` SMPL-X viewer) → retarget → visualize pipeline (robots: `unitree_h1`, `bdx`).
> `main`: every original Kimodo / GMR feature (other robots, BVH/FBX/OptiTrack inputs, demo UI, ...).

## Layout

```
kimodo/                      Kimodo package (text-to-motion, SMPL-X output)
  demo/, viz/                kimodo_demo web UI (SMPL-X motion viewer)
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
cd C:\kimodo
pip install -e .
```

Every command below assumes `conda activate env_kimodo` and the `C:\kimodo` working directory.

`MotionCorrection` is built with CMake on install. If a prebuilt
`MotionCorrection/python/motion_correction/_motion_correction*.pyd` already exists, it is reused and no build runs.

## Usage

### 1. Create motion data

```bash
kimodo_gen "A person stands still." --duration 6 --output motion_data\stand
```

Writes `motion_data\stand.npz` and `motion_data\stand_amass.npz`. The `_amass.npz` file is the GMR input.
The model defaults to `Kimodo-SMPLX-RP-v1`.

#### Text encoder server (recommended)

By default `kimodo_gen` loads the 8B text encoder (LLM2Vec, Llama-3-8B) on every run, which takes several minutes.
Keep it loaded in a separate terminal instead:

```bash
# terminal 1: start once and leave it running (ready when port 9550 is listening)
conda activate env_kimodo
cd C:\kimodo
kimodo_textencoder

# terminal 2: kimodo_gen / kimodo_demo connect to it automatically
kimodo_gen "A person stands still." --duration 6 --output motion_data\stand
```

- `kimodo_gen` first tries `http://127.0.0.1:9550/`. If no server is running it prints
  `Text encoder service is unreachable, falling back to local LLM2Vec encoder` and loads the encoder itself.
- Force a mode with `set TEXT_ENCODER_MODE=api` (server only) or `set TEXT_ENCODER_MODE=local` (never use the server).
- The server listens on port 9550 (change with `set GRADIO_SERVER_PORT=...`, and point clients at it with
  `set TEXT_ENCODER_URL=http://127.0.0.1:<port>/`).

### 1-1. (Optional) View the motion on the SMPL-X body (kimodo_demo)

```bash
conda activate env_kimodo
cd C:\kimodo
kimodo_demo
```

1. Open http://127.0.0.1:7860 in a browser. The `Kimodo-SMPLX-RP-v1` model loads by default
   (`--model` picks another one). The first launch takes a while because the model and text encoder load.
2. Open **Load/Save → Motion** in the left panel.
3. Set **Load Path** to the motion file, e.g. `C:\kimodo\motion_data\stand.npz`
   (Kimodo `.npz` or AMASS `_amass.npz` both work), then click **Load Motion**.
4. Use the timeline at the bottom to play or scrub the motion.

- The SMPL-X mesh uses `assets/body_models/smplx/SMPLX_NEUTRAL.npz` (the same file GMR uses).
- Only motions whose skeleton matches the loaded model can be loaded, so keep the SMPLX model selected.
- You can also type a prompt in **Generate** to create, preview and save motions directly from the UI.
- Change the port with the `SERVER_PORT` environment variable (`set SERVER_PORT=7861`).

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
