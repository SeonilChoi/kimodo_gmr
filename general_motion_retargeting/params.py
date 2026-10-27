import pathlib

HERE = pathlib.Path(__file__).parent
IK_CONFIG_ROOT = HERE / "ik_configs"
ASSET_ROOT = HERE / ".." / "assets"

# To add a robot: generate its MuJoCo model with urdf_to_gmr, derive its IK config with
# scripts/calibrate_smplx_to_robot.py, and register it in every dict below.
ROBOT_XML_DICT = {
    "unitree_g1": ASSET_ROOT / "unitree_g1" / "g1_mocap_29dof.xml",
    "bdx": ASSET_ROOT / "bdx" / "bdx.xml",
    "arti_d1": ASSET_ROOT / "arti_d1" / "arti_d1.xml",
}

IK_CONFIG_DICT = {
    "smplx": {
        "unitree_g1": IK_CONFIG_ROOT / "smplx_to_g1.json",
        "bdx": IK_CONFIG_ROOT / "smplx_to_bdx.json",
        "arti_d1": IK_CONFIG_ROOT / "smplx_to_arti_d1.json",
    },
}

ROBOT_BASE_DICT = {
    "unitree_g1": "pelvis",
    "bdx": "pelvis",
    "arti_d1": "trunk",  # pelvis origin sits on the floor; follow the torso instead
}

VIEWER_CAM_DISTANCE_DICT = {
    "unitree_g1": 2.0,
    "bdx": 2.0,
    "arti_d1": 1.5,
}
