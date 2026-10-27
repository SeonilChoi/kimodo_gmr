"""Command line entry point: ``urdf_to_gmr`` / ``python -m urdf_to_gmr``."""

import argparse
from pathlib import Path

from .converter import Frame, convert_urdf


def _parse_frame(value: str) -> Frame:
    # NAME:PARENT or NAME:PARENT:x,y,z
    parts = value.split(":")
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError(f"--frame expects NAME:PARENT[:x,y,z], got '{value}'")
    pos = (0.0, 0.0, 0.0)
    if len(parts) == 3:
        try:
            pos = tuple(float(v) for v in parts[2].split(","))
        except ValueError:
            pos = ()
        if len(pos) != 3:
            raise argparse.ArgumentTypeError(f"--frame position must be x,y,z, got '{parts[2]}'")
    return Frame(name=parts[0], parent=parts[1], pos=pos)


def parse_args():
    parser = argparse.ArgumentParser(
        prog="urdf_to_gmr",
        description="Convert a robot URDF into a MuJoCo XML for SMPL-X -> robot retargeting with GMR.",
    )
    parser.add_argument("urdf", type=Path, help="Robot URDF. Its root link becomes the floating base.")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output XML (default: <urdf dir>/../<robot name>.xml, i.e. assets/<robot>/<robot>.xml).",
    )
    parser.add_argument(
        "--frame",
        type=_parse_frame,
        action="append",
        default=[],
        metavar="NAME:PARENT[:x,y,z]",
        help="Add an IK target body NAME under body PARENT at offset x,y,z (repeatable).",
    )
    parser.add_argument(
        "--package-dir",
        type=Path,
        action="append",
        default=[],
        help="Directory used to resolve package:// mesh URIs (repeatable).",
    )
    parser.add_argument("--name", default=None, help="MuJoCo model name (default: URDF robot name).")
    parser.add_argument(
        "--root-height",
        type=float,
        default=None,
        help="Root body height at qpos = 0 (default: place the lowest point of the robot on z = 0).",
    )
    parser.add_argument("--no-collision", action="store_true", help="Drop collision geoms (keep visual meshes only).")
    parser.add_argument("--no-scene", action="store_true", help="Do not add floor, light and home keyframe.")
    return parser.parse_args()


def main():
    args = parse_args()
    output = args.output
    if output is None:
        import xml.etree.ElementTree as ET

        robot_name = args.name or ET.parse(args.urdf).getroot().get("name") or args.urdf.stem
        output = args.urdf.resolve().parent.parent / f"{robot_name}.xml"

    result = convert_urdf(
        args.urdf,
        output,
        frames=args.frame,
        package_dirs=args.package_dir,
        model_name=args.name,
        root_height=args.root_height,
        collision=not args.no_collision,
        scene=not args.no_scene,
    )

    robot = result.xml_path.stem
    print(f"Saved {result.xml_path}")
    print(f"  root body : {result.root_body} (height {result.root_height:.4f} m at qpos = 0)")
    print(f"  joints    : {len(result.joint_names)} -> {', '.join(result.joint_names)}")
    print(f"  bodies    : {', '.join(result.body_names)}")
    print()
    print("Next steps to retarget onto this robot:")
    print(f"  1. Write general_motion_retargeting/ik_configs/smplx_to_{robot}.json using the body names above.")
    print("  2. Register the robot in general_motion_retargeting/params.py:")
    print(f'       ROBOT_XML_DICT["{robot}"]  = ASSET_ROOT / ... / "{result.xml_path.name}"')
    print(f'       IK_CONFIG_DICT["smplx"]["{robot}"] = IK_CONFIG_ROOT / "smplx_to_{robot}.json"')
    print(f'       ROBOT_BASE_DICT["{robot}"] = "{result.root_body}"')
    print(f'       VIEWER_CAM_DISTANCE_DICT["{robot}"] = 2.0')


if __name__ == "__main__":
    main()
