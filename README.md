# Kimodo & GMR

Text → human motion (Kimodo, SMPL-X) → humanoid robot motion (GMR).

> `develop`: only the generate (+ `kimodo_demo` SMPL-X viewer) → retarget → visualize pipeline (robots: `unitree_g1`, `bdx`, `arti_d1`).
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
  calibrate_smplx_to_robot.py  Derive a robot's SMPL-X IK config from its MuJoCo XML
urdf_to_gmr/                 URDF → GMR-ready MuJoCo XML generator (for adding new robots)
assets/
  unitree_g1/                Unitree G1 MuJoCo model
  bdx/, arti_d1/             MuJoCo model (<robot>.xml) + urdf/ + meshes/
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
python scripts\smplx_to_robot.py --smplx_file motion_data\stand_amass.npz --robot unitree_g1 --save_path outputs\g1_stand.pkl --rate_limit
```

### 3. Visualize motion data

```bash
python scripts\vis_robot_motion.py --robot unitree_g1 --robot_motion_path outputs\g1_stand.pkl
```

`--robot` is `unitree_g1`, `bdx` or `arti_d1`. For another robot, change `--robot` (e.g. `--robot bdx --save_path outputs\bdx_stand.pkl`).

## Adding a robot

### 1. URDF → MuJoCo XML (`urdf_to_gmr`)

`urdf_to_gmr` builds the MuJoCo XML GMR needs (like `assets/unitree_g1/g1_mocap_29dof.xml`) from a robot URDF.
Lay the robot out like the existing ones:

```
assets/<robot>/
  urdf/<robot>.urdf          mesh paths relative to the URDF, e.g. ../meshes/pelvis.stl
  meshes/*.stl
```

```bash
urdf_to_gmr assets\<robot>\urdf\<robot>.urdf
# -> assets\<robot>\<robot>.xml
```

What it does:
- Compiles the URDF with MuJoCo's own importer, so links, joint axes/limits, inertias and meshes match the URDF exactly.
- Adds a free joint to the URDF root link (the GMR base body) and lifts it so the robot stands on z = 0 at qpos = 0.
- Keeps fixed URDF links as bodies so they can be IK targets, and adds one motor per joint, a floor, lights and a
  `home` keyframe.
- Writes mesh paths relative to the output file and loads the result once to check the layout GMR expects.

Options:

| Option | Meaning |
| --- | --- |
| `-o PATH` | Output file (default `assets/<robot>/<robot name>.xml`) |
| `--frame NAME:PARENT[:x,y,z]` | Add an IK target body (e.g. a toe) under `PARENT` (repeatable) |
| `--root-height Z` | Fix the root height instead of placing the lowest point on the floor |
| `--package-dir DIR` | Resolve `package://` mesh URIs (repeatable) |
| `--name NAME` | MuJoCo model name (default: URDF robot name) |
| `--no-collision` / `--no-scene` | Drop collision geoms / floor, light and keyframe |

Example (BDX with the toe/heel frames used by `assets/bdx/bdx.xml`):

```bash
urdf_to_gmr assets\bdx\urdf\bdx.urdf -o assets\bdx\bdx_generated.xml ^
  --frame left_toe:left_foot_link:0.112,0,-0.0785 --frame right_toe:right_foot_link:0.112,0,-0.0785
```

It also works from Python: `from urdf_to_gmr import convert_urdf, Frame`.

### 2. IK config (`scripts/calibrate_smplx_to_robot.py`)

The IK config is derived from the robot XML instead of written by hand. The robot's zero pose is matched to the
SMPL-X rest pose, so standing still retargets to the robot's zero pose with zero IK error.

1. Add an entry to `ROBOTS` in `scripts/calibrate_smplx_to_robot.py`. Give it the XML, the root body, the foot
   bodies (`sole_bodies`), and a `spec` that maps robot bodies to SMPL-X joints with position/rotation weights
   (use the body names `urdf_to_gmr` printed; `bdx` / `arti_d1` are templates).
   If a limb's zero pose is not the SMPL-X T-pose (e.g. arms that rest down or along the body), add the
   human joint's reference rotation as a 5th spec item; `arti_d1`'s arms do this.
2. Run it (it also checks that the rest pose is a zero-error fixed point):
   ```bash
   python scripts\calibrate_smplx_to_robot.py <robot>
   # -> general_motion_retargeting/ik_configs/smplx_to_<robot>.json
   ```

### 3. Register

Add the robot to every dict in `general_motion_retargeting/params.py` (`urdf_to_gmr` prints the entries).
After that, `--robot <robot>` works in `smplx_to_robot.py` and `vis_robot_motion.py`.

The Unitree G1 config (`smplx_to_g1.json`) is the original hand-tuned GMR config, not generated by this script.
