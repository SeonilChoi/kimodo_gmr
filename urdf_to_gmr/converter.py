"""Convert a robot URDF into a MuJoCo XML that GMR can retarget SMPL-X motion onto.

The kinematic tree, inertias, joint axes/limits and meshes are compiled by MuJoCo's own
URDF importer, so they match the URDF exactly. On top of that this module adds what GMR
needs (see general_motion_retargeting/motion_retarget.py and robot_motion_viewer.py):

- a free joint on the root body, so qpos = [root pos (3), root quat wxyz (4), joint angles]
- the root body lifted so the robot stands on z = 0 at qpos = 0
- fixed URDF links kept as bodies, so they can be used as IK frames (e.g. foot/toe frames)
- optional extra frame bodies for IK targets that the URDF does not define
- one motor per joint, a floor/light scene and a "home" keyframe
- mesh paths relative to the output file (compiler meshdir), like the other assets/ models
"""

import os
import re
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import mujoco as mj
import numpy as np

FREEJOINT_NAME = "floating_base"


@dataclass
class Frame:
    """An extra body (IK target frame) attached to an existing body."""

    name: str
    parent: str
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class ConversionResult:
    xml_path: Path
    root_body: str
    root_height: float
    joint_names: list[str] = field(default_factory=list)
    body_names: list[str] = field(default_factory=list)


def _resolve_mesh_path(filename: str, urdf_dir: Path, package_dirs: list[Path]) -> Path:
    if filename.startswith("package://"):
        package, _, rel = filename[len("package://") :].partition("/")
        roots = list(package_dirs) + list(urdf_dir.parents)[:3]
        candidates = [root / package / rel for root in roots] + [root / rel for root in roots]
    elif filename.startswith("file://"):
        candidates = [Path(filename[len("file://") :])]
    else:
        candidates = [urdf_dir / filename]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(f"Mesh '{filename}' not found (looked in: {', '.join(str(c) for c in candidates)})")


def _prepare_urdf(urdf_path: Path, package_dirs: list[Path]) -> str:
    """Make mesh paths absolute and tell MuJoCo to keep visuals and fixed links."""
    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    if robot.tag != "robot":
        raise ValueError(f"{urdf_path} is not a URDF (root tag <{robot.tag}>)")

    for mesh in robot.iter("mesh"):
        path = _resolve_mesh_path(mesh.get("filename", ""), urdf_path.parent, package_dirs)
        if path.suffix.lower() not in (".stl", ".obj", ".msh"):
            raise ValueError(f"MuJoCo cannot load '{path.name}': convert it to .stl or .obj first")
        mesh.set("filename", path.as_posix())

    for old in robot.findall("mujoco"):
        robot.remove(old)
    mujoco_ext = ET.Element("mujoco")
    ET.SubElement(
        mujoco_ext,
        "compiler",
        discardvisual="false",
        fusestatic="false",
        strippath="false",
        balanceinertia="true",
    )
    robot.insert(0, mujoco_ext)
    return ET.tostring(robot, encoding="unicode")


def _compile_urdf(urdf_xml: str) -> ET.Element:
    """Compile the URDF with MuJoCo and return the equivalent MJCF tree."""
    with tempfile.TemporaryDirectory() as tmp:
        urdf_tmp = os.path.join(tmp, "robot.urdf")
        mjcf_tmp = os.path.join(tmp, "robot.xml")
        with open(urdf_tmp, "w", encoding="utf-8") as f:
            f.write(urdf_xml)
        model = mj.MjModel.from_xml_path(urdf_tmp)
        mj.mj_saveLastXML(mjcf_tmp, model)
        return ET.parse(mjcf_tmp).getroot()


def _relativize_meshes(mjcf: ET.Element, out_dir: Path) -> None:
    meshes = mjcf.findall("./asset/mesh")
    if not meshes:
        return
    files = [Path(mesh.get("file")) for mesh in meshes]
    try:
        mesh_root = Path(os.path.commonpath([f.parent for f in files]))
        meshdir = os.path.relpath(mesh_root, out_dir)
    except ValueError:  # different drives on Windows: keep absolute paths
        return
    for mesh, f in zip(meshes, files):
        mesh.set("file", f.relative_to(mesh_root).as_posix())
        mesh.attrib.pop("content_type", None)
    mjcf.find("compiler").set("meshdir", Path(meshdir).as_posix())


def _find_body(mjcf: ET.Element, name: str) -> ET.Element:
    for body in mjcf.iter("body"):
        if body.get("name") == name:
            return body
    raise ValueError(f"Body '{name}' not found in the model")


def _insert_after_inertial(body: ET.Element, element: ET.Element) -> None:
    index = 1 if len(body) and body[0].tag == "inertial" else 0
    body.insert(index, element)


def _add_floating_base(mjcf: ET.Element) -> ET.Element:
    roots = mjcf.find("worldbody").findall("body")
    if len(roots) != 1:
        names = [b.get("name") for b in roots]
        raise ValueError(f"Expected one root link attached to the world, found {len(roots)}: {names}")
    root = roots[0]
    if root.find("joint") is not None or root.find("freejoint") is not None:
        raise ValueError(f"Root link '{root.get('name')}' already has a joint; the URDF root must be the floating base")
    _insert_after_inertial(root, ET.Element("freejoint", name=FREEJOINT_NAME))
    return root


def _add_frames(mjcf: ET.Element, frames: list[Frame]) -> None:
    existing = {body.get("name") for body in mjcf.iter("body")}
    for frame in frames:
        if frame.name in existing:
            raise ValueError(f"Frame '{frame.name}' already exists in the model")
        parent = _find_body(mjcf, frame.parent)
        ET.SubElement(parent, "body", name=frame.name, pos=" ".join(f"{v:g}" for v in frame.pos))
        existing.add(frame.name)


def _add_actuators(mjcf: ET.Element) -> list[str]:
    joint_names = [j.get("name") for j in mjcf.find("worldbody").iter("joint")]
    actuator = ET.SubElement(mjcf, "actuator")
    for name in joint_names:
        motor = ET.SubElement(actuator, "motor", name=name, joint=name)
        joint = next(j for j in mjcf.iter("joint") if j.get("name") == name)
        if joint.get("actuatorfrcrange"):
            motor.set("ctrlrange", joint.get("actuatorfrcrange"))
    return joint_names


def _add_scene(mjcf: ET.Element, n_joints: int) -> None:
    visual = ET.SubElement(mjcf, "visual")
    ET.SubElement(visual, "headlight", diffuse="0.6 0.6 0.6", ambient="0.1 0.1 0.1", specular="0.9 0.9 0.9")
    ET.SubElement(visual, "rgba", haze="0.15 0.25 0.35 1")
    ET.SubElement(visual, "global", azimuth="-140", elevation="-20", offwidth="2080", offheight="1170")

    asset = mjcf.find("asset")
    if asset is None:
        asset = ET.SubElement(mjcf, "asset")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient", rgb1="1 1 1", rgb2="1 1 1", width="800", height="800")
    ET.SubElement(
        asset,
        "texture",
        type="2d",
        name="groundplane",
        builtin="checker",
        mark="edge",
        rgb1="1 1 1",
        rgb2="1 1 1",
        markrgb="0 0 0",
        width="300",
        height="300",
    )
    ET.SubElement(asset, "material", name="groundplane", texture="groundplane", texuniform="true", texrepeat="5 5", reflectance="0")

    worldbody = mjcf.find("worldbody")
    worldbody.insert(
        0,
        ET.Element(
            "geom",
            name="floor",
            size="0 0 0.01",
            type="plane",
            material="groundplane",
            contype="1",
            conaffinity="0",
            priority="1",
            friction="0.6",
            condim="3",
        ),
    )
    worldbody.insert(1, ET.Element("light", diffuse=".5 .5 .5", pos="-3 -3 5", dir="3 3 -5", castshadow="true"))

    keyframe = ET.SubElement(mjcf, "keyframe")
    ET.SubElement(keyframe, "key", name="home", qpos=" ".join(["0 0 0 1 0 0 0"] + ["0"] * n_joints))


def _lowest_point(model: mj.MjModel) -> float:
    """Lowest z of all robot geoms at qpos = 0 (mesh vertices, or bounding spheres for primitives)."""
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    lowest = np.inf
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == 0:  # floor and other world geoms
            continue
        if model.geom_type[g] == mj.mjtGeom.mjGEOM_MESH:
            mesh_id = model.geom_dataid[g]
            start, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
            verts = model.mesh_vert[start : start + count]
            z = data.geom_xpos[g][2] + verts @ data.geom_xmat[g].reshape(3, 3)[2]
            lowest = min(lowest, float(z.min()))
        else:
            lowest = min(lowest, float(data.geom_xpos[g][2] - model.geom_rbound[g]))
    return lowest


def _write(mjcf: ET.Element, path: Path, header: str) -> None:
    ET.indent(mjcf, space="  ")
    body = ET.tostring(mjcf, encoding="unicode")
    path.write_text(f"<!--\n{header}\n-->\n{body}\n", encoding="utf-8")


def convert_urdf(
    urdf_path: str | Path,
    output_path: str | Path,
    frames: list[Frame] | None = None,
    package_dirs: list[str | Path] | None = None,
    model_name: str | None = None,
    root_height: float | None = None,
    collision: bool = True,
    scene: bool = True,
) -> ConversionResult:
    """Convert ``urdf_path`` into a GMR-ready MuJoCo XML at ``output_path``.

    Args:
        urdf_path: Robot URDF. Its root link becomes the floating base (GMR root body).
        output_path: Where to write the MuJoCo XML. Mesh paths are written relative to it.
        frames: Extra bodies to add as IK targets (e.g. a toe or head frame).
        package_dirs: Directories used to resolve ``package://`` mesh URIs.
        model_name: Name of the MuJoCo model (defaults to the URDF robot name).
        root_height: Height of the root body at qpos = 0. ``None`` puts the lowest point on z = 0.
        collision: Keep collision geoms (visual-only geoms are always kept).
        scene: Add floor, light, skybox and a "home" keyframe.
    """
    urdf_path = Path(urdf_path).resolve()
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    mjcf = _compile_urdf(_prepare_urdf(urdf_path, [Path(p).resolve() for p in package_dirs or []]))
    if model_name:
        mjcf.set("model", model_name)
    compiler = mjcf.find("compiler")
    compiler.set("autolimits", "true")
    # Zero-mass URDF links (frames such as "trunk") are fine for kinematics; keep MuJoCo from rejecting them.
    compiler.set("boundmass", "1e-6")
    compiler.set("boundinertia", "1e-9")

    if not collision:
        for body in mjcf.iter("body"):
            for geom in body.findall("geom"):
                if geom.get("contype") != "0":
                    body.remove(geom)

    _relativize_meshes(mjcf, output_path.parent)
    root = _add_floating_base(mjcf)
    _add_frames(mjcf, frames or [])
    joint_names = _add_actuators(mjcf)
    if scene:
        _add_scene(mjcf, len(joint_names))

    header = (
        f"  Generated by urdf_to_gmr from {Path(os.path.relpath(urdf_path, output_path.parent)).as_posix()}.\n"
        f"  Root body (GMR base): {root.get('name')}, free joint: {FREEJOINT_NAME}.\n"
        "  Kinematics, inertias, joint limits and meshes are compiled from the URDF by MuJoCo."
    )
    _write(mjcf, output_path, header)

    # Lift the root so the robot stands on the floor at qpos = 0.
    if root_height is None:
        model = mj.MjModel.from_xml_path(str(output_path))
        root_pos = np.array([float(v) for v in (root.get("pos") or "0 0 0").split()])
        root_height = float(root_pos[2] - _lowest_point(model))
        root_pos[2] = root_height
    else:
        root_pos = np.array([float(v) for v in (root.get("pos") or "0 0 0").split()])
        root_pos[2] = root_height
    root.set("pos", " ".join(f"{v:.6g}" for v in root_pos))
    if scene:
        key = mjcf.find("./keyframe/key")
        key.set("qpos", re.sub(r"^\S+ \S+ \S+", " ".join(f"{v:.6g}" for v in root_pos), key.get("qpos")))
    _write(mjcf, output_path, header)

    model = mj.MjModel.from_xml_path(str(output_path))
    if model.nq != 7 + len(joint_names) or model.jnt_type[0] != mj.mjtJoint.mjJNT_FREE:
        raise RuntimeError("Generated model does not have the free joint + 1-DoF joints layout GMR expects")

    return ConversionResult(
        xml_path=output_path,
        root_body=root.get("name"),
        root_height=root_height,
        joint_names=joint_names,
        body_names=[mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) for i in range(1, model.nbody)],
    )
