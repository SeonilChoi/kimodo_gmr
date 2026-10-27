"""Regenerate a SMPL-X -> robot IK config from the robot's own MJCF.

    python scripts/calibrate_smplx_to_robot.py bdx
    python scripts/calibrate_smplx_to_robot.py arti_d1
    python scripts/calibrate_smplx_to_robot.py            # all of the above

To add a robot: generate its MJCF with urdf_to_gmr, add an entry to ROBOTS below
(robot body <- SMPL-X joint + weights), run this script, then register the robot in
general_motion_retargeting/params.py.

Run from the repo root. Writing the config by hand is error prone and it silently
rots whenever the MJCF moves, so it is derived instead.

Calibration contract
--------------------
GMR builds the IK target for robot body B as

    target_pos = s_root * p_pelvis + s_B * (p_B - p_pelvis) + R_target * o_B
    target_rot = R_human_B * q_B                                  (= R_target)

Take the reference pose to be the SMPL-X rest pose of the neutral template. It
already stands with both soles flat on the floor (the ankle sits 0.076 m above
the mesh floor and the ball of the foot 0.018 m), and every joint rotation in it
is identity. Requiring target == the robot's own rest pose at that reference:

    q_B = Rc . R_rest(B)            Rc: template axes -> robot axes (x fwd, y left, z up)
    s_B = |r_B| / |u_B|             radial scale, so segment reach is preserved
    o_B = R_rest(B)^-1 . (r_B - s_B * u_B)      residual, in B's own frame
    s_root = (z_trunk - z_sole) / h_pelvis      so the soles land on z = 0

r_B is the robot body's rest offset from its root frame, u_B the human joint's
rest offset from the pelvis. The rest pose is then an exact zero-error fixed
point of the IK problem, in position and orientation alike, whatever the robot's
link orientations happen to be at rest.

The SMPL-X template is slightly asymmetric left-to-right; that asymmetry is
averaged out of u_B so it is not baked into the robot. The robot's own geometry
is used exactly as the MJCF states it, asymmetries included.

Weights: a leg with N DoF can serve foot position (3) plus at most N-3 more
constraints, so thigh/shin orientation targets mostly just fight the position
tasks. Root and feet are therefore tracked hard and the limb targets are weak
tiebreakers. Verified by sweeping on AMASS clips; see the notes per robot below.
"""
import collections
import json
import sys

import mujoco as mj
import numpy as np
import smplx
import torch
from scipy.spatial.transform import Rotation as R
from smplx.joint_names import JOINT_NAMES

H_TPL, H_ASSUM = 1.66, 1.8               # GMR's own height model maps betas=0 -> 1.66 m
CANON = np.array([-0.5, 0.5, 0.5, 0.5])  # wxyz: template (x left, y up, z fwd) -> robot (x fwd, y left, z up)
Rc = R.from_quat(CANON, scalar_first=True)
MIRROR = np.array([1.0, -1.0, 1.0])
cvt = lambda p: np.array([p[2], p[0], p[1]])
sym = lambda n: n.replace("left_", "").replace("right_", "")
rnd = lambda v: [round(float(x), 6) for x in v]
# Human arms hanging relaxed: upper-arm (shoulder joint) rotation relative to the pelvis, as a
# rotation vector in template axes. Measured as the mean over Kimodo-SMPLX-RP "A person stands
# still." motions and left/right symmetrised; a plain 80 deg drop about the forward axis leaves
# ~60 deg of residual twist/flexion that the IK then pushes into the wrong arm joint.
ARM_DOWN_L = (0.7643, -0.7609, -1.0315)
ARM_DOWN_R = (0.7643, 0.7609, 1.0315)

# robot body <- SMPL-X joint, with (pos, rot) weights for ik_match_table1 / table2
ROBOTS = {
    "bdx": dict(
        xml="assets/bdx/bdx.xml", root="trunk",
        sole_bodies=("left_foot_link", "right_foot_link"),
        # 5 DoF per leg, pitch-only ankle, no arms; head is BDX's expressive DoF
        spec=[("trunk",           "pelvis",      (300, 40), (300, 40)),
              ("left_thigh",      "left_hip",    (  0,  5), (  0,  2)),
              ("right_thigh",     "right_hip",   (  0,  5), (  0,  2)),
              ("left_shin",       "left_knee",   (  0,  5), (  0,  2)),
              ("right_shin",      "right_knee",  (  0,  5), (  0,  2)),
              ("left_foot_link",  "left_ankle",  (  0, 10), (  5, 10)),
              ("right_foot_link", "right_ankle", (  0, 10), (  5, 10)),
              ("left_foot",       "left_foot",   (300, 10), (300, 10)),
              ("right_foot",      "right_foot",  (300, 10), (300, 10)),
              ("neck",            "neck",        (  0,  2), (  0,  2)),
              ("head",            "head",        (  0, 20), (  0, 10))],
    ),
    "arti_d1": dict(
        xml="assets/arti_d1/arti_d1.xml", root="trunk",
        sole_bodies=("left_foot_link", "right_foot_link"),
        # Same leg layout as BDX (5 DoF, pitch-only ankle), no head. The two 2-DoF arm
        # plates stick out sideways at rest but can barely move down (-13 deg) and mostly
        # up (+67 deg), so they are calibrated against the human arms hanging relaxed
        # (ARM_DOWN_*): arms down -> plates at rest, arms raised -> plates raised.
        # Weak rotation-only targets.
        spec=[("trunk",           "pelvis",         (300, 40), (300, 40)),
              ("left_thigh",      "left_hip",       (  0,  5), (  0,  2)),
              ("right_thigh",     "right_hip",      (  0,  5), (  0,  2)),
              ("left_shin",       "left_knee",      (  0,  5), (  0,  2)),
              ("right_shin",      "right_knee",     (  0,  5), (  0,  2)),
              ("left_foot_link",  "left_ankle",     (  0, 10), (  5, 10)),
              ("right_foot_link", "right_ankle",    (  0, 10), (  5, 10)),
              ("left_foot",       "left_foot",      (300, 10), (300, 10)),
              ("right_foot",      "right_foot",     (300, 10), (300, 10)),
              ("left_arm_plate",  "left_shoulder",  (  0,  2), (  0,  2), ARM_DOWN_L),
              ("right_arm_plate", "right_shoulder", (  0,  2), (  0,  2), ARM_DOWN_R)],
    ),
}


def smplx_reference():
    """SMPL-X neutral template at its rest pose: joint offsets from the pelvis (robot
    axes, left/right symmetrised) and the pelvis height above the mesh floor."""
    bm = smplx.create("assets/body_models", "smplx", gender="neutral", use_pca=False,
                      num_betas=16, num_expression_coeffs=10)
    out = bm(betas=torch.zeros(1, 16), global_orient=torch.zeros(1, 3),
             body_pose=torch.zeros(1, 63), transl=torch.zeros(1, 3),
             left_hand_pose=torch.zeros(1, 45), right_hand_pose=torch.zeros(1, 45),
             jaw_pose=torch.zeros(1, 3), leye_pose=torch.zeros(1, 3),
             reye_pose=torch.zeros(1, 3), return_full_pose=True)
    J = out.joints[0].detach().numpy()
    names = list(JOINT_NAMES[: len(bm.parents)])
    floor = out.vertices[0].detach().numpy()[:, 1].min()
    u = {n: cvt(J[names.index(n)] - J[names.index("pelvis")]) for n in names}
    for n in [n for n in names if n.startswith("left_")]:
        r_ = "right_" + n[5:]
        if r_ in u:
            avg = (u[n] + u[r_] * MIRROR) / 2
            u[n], u[r_] = avg, avg * MIRROR
    return u, float(J[names.index("pelvis")][1] - floor)


def calibrate(name, cfg, u_all, h_pelvis, verbose=True):
    m = mj.MjModel.from_xml_path(cfg["xml"])
    d = mj.MjData(m)
    d.qpos[:] = m.key_qpos[0] if m.nkey else 0
    mj.mj_forward(m, d)
    bid = {mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, i): i for i in range(m.nbody)}
    root = d.xpos[bid[cfg["root"]]].copy()

    # lowest mesh vertex of the foot bodies (works for unnamed geoms, e.g. urdf_to_gmr output)
    sole_ids = {bid[b] for b in cfg["sole_bodies"]}
    sole = min(
        (((d.geom_xmat[g].reshape(3, 3) @ m.mesh_vert[
            m.mesh_vertadr[m.geom_dataid[g]]:
            m.mesh_vertadr[m.geom_dataid[g]] + m.mesh_vertnum[m.geom_dataid[g]]].T).T[:, 2]
          + d.geom_xpos[g][2]).min())
        for g in range(m.ngeom)
        if m.geom_bodyid[g] in sole_ids and m.geom_type[g] == mj.mjtGeom.mjGEOM_MESH)

    pairs = [(b, h) for b, h, *_ in cfg["spec"]]
    # Optional 5th spec entry: rotation vector (rad, template axes) of the human joint in the
    # reference pose, for limbs whose robot rest pose is not the SMPL-X rest (T-)pose.
    A = {h: R.from_rotvec(rest[0] if rest else np.zeros(3)) for _, h, _, _, *rest in cfg["spec"]}
    u = {h: u_all[h] for _, h in pairs}
    r = {h: d.xpos[bid[b]] - root for b, h in pairs}
    Rrest = {h: R.from_matrix(d.xmat[bid[b]].reshape(3, 3)) for b, h in pairs}

    q = {h: A[h].inv() * Rc * Rrest[h] for _, h in pairs}         # rotation offset
    s, o = {}, {}
    for _, h in pairs:
        if h == "pelvis":
            s[h], o[h] = (root[2] - sole) / h_pelvis, np.zeros(3)
        else:
            s[h] = float(np.linalg.norm(r[h]) / np.linalg.norm(u[h]))
            o[h] = Rrest[h].inv().apply(r[h] - s[h] * u[h])

    def table(k):
        return {b: [h, cfg["spec"][i][2 + k][0], cfg["spec"][i][2 + k][1],
                    rnd(o[h]), rnd(q[h].as_quat(scalar_first=True))]
                for i, (b, h) in enumerate(pairs)}

    out = {"robot_root_name": cfg["root"], "human_root_name": "pelvis", "ground_height": 0.0,
           "human_height_assumption": H_ASSUM, "use_ik_match_table1": True,
           "use_ik_match_table2": True,
           "human_scale_table": {h: round(s[h] * H_TPL / H_ASSUM, 6) for _, h in pairs},
           "ik_match_table1": table(0), "ik_match_table2": table(1)}
    path = f"general_motion_retargeting/ik_configs/smplx_to_{name}.json"
    json.dump(out, open(path, "w"), indent=2)

    if verbose:
        print(f"\n=== {name} ===  root {cfg['root']} at z={root[2]:.4f}, soles at z={sole:+.4f}")
        print(f"{'robot body':22s} {'<- smplx':13s} {'scale':>8s}  {'pos offset (m)':26s} rot offset (wxyz)")
        for b, h in pairs:
            print(f"{b:22s} {h:13s} {s[h] * H_TPL / H_ASSUM:8.4f}  {str(np.round(o[h], 4)):26s} "
                  f"{np.round(q[h].as_quat(scalar_first=True), 4)}")
        worst_p = worst_r = 0.0
        for b, h in pairs:
            Rt = Rc.inv() * A[h] * q[h]                            # R_human(ref) . q, R_human(ref)=Rc^-1 . A
            tgt = np.zeros(3) if h == "pelvis" else s[h] * u[h] + Rt.apply(o[h])
            worst_p = max(worst_p, float(np.linalg.norm(tgt - r[h])))
            worst_r = max(worst_r, float((Rt.inv() * Rrest[h]).magnitude()))
        print(f"rest-pose fixed point: worst pos {worst_p * 1000:.3f} mm, worst rot {worst_r:.2e} rad")
        print(f"wrote {path}")
    return out


if __name__ == "__main__":
    which = sys.argv[1:] or list(ROBOTS)
    u_all, h_pelvis = smplx_reference()
    print(f"SMPL-X reference: pelvis {h_pelvis:.4f} m above the mesh floor "
          f"(GMR height model: {H_TPL} m)")
    for n in which:
        calibrate(n, ROBOTS[n], u_all, h_pelvis)
