"""MuscleMimic Models — MuJoCo musculoskeletal model package."""

from pathlib import Path
import mujoco

MODELS_DIR = Path(__file__).resolve().parent / "model"

REGISTRY = {
    "bimanual": "arm/myoarm_bimanual.xml",
    "fullbody": "body/myofullbody.xml",
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
