from pathlib import Path
from importlib.metadata import version

import mujoco

__version__ = version("musclemimic-models")

MODELS_DIR = Path(__file__).resolve().parent / "model"

REGISTRY = {
    "bimanual": "arm/myoarm_bimanual.xml",
    "bimanual_pre_mod": "arm/myoarm_bimanual_pre_mod.xml",
    "myofullbody": "body/myofullbody.xml",
    "myofullbody_pre_mod": "body/myofullbody_pre_mod.xml",
}


def get_xml_path(name):
    """Get absolute path to a model XML. Raises ValueError if unknown."""
    if name not in REGISTRY:
        raise ValueError(f"Unknown model {name!r}. Choose from: {list(REGISTRY)}")
    return MODELS_DIR / REGISTRY[name]


def load(name):
    """Load a MuJoCo model by name. Returns (MjModel, MjData)."""
    xml_path = get_xml_path(name)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    return model, mujoco.MjData(model)


def print_path(name=None):
    """Print model path(s). If name is None, print all."""
    if name:
        print(get_xml_path(name))
    else:
        for k in REGISTRY:
            print(f"{k}: {MODELS_DIR / REGISTRY[k]}")
