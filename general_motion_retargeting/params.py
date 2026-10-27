import pathlib

HERE = pathlib.Path(__file__).parent
IK_CONFIG_ROOT = HERE / "ik_configs"
ASSET_ROOT = HERE / ".." / "assets"

# To add a robot: put its MuJoCo model under assets/<robot>/, add an IK config
# under ik_configs/, and register it in every dict below.
ROBOT_XML_DICT = {
    "unitree_g1": ASSET_ROOT / "unitree_g1" / "g1_mocap_29dof.xml",
    "bdx": ASSET_ROOT / "bdx" / "bdx.xml",
}

IK_CONFIG_DICT = {
    "smplx": {
        "unitree_g1": IK_CONFIG_ROOT / "smplx_to_g1.json",
        "bdx": IK_CONFIG_ROOT / "smplx_to_bdx.json",
    },
}

ROBOT_BASE_DICT = {
    "unitree_g1": "pelvis",
    "bdx": "pelvis",
}

VIEWER_CAM_DISTANCE_DICT = {
    "unitree_g1": 2.0,
    "bdx": 2.0,
}
