"""Compare full-body mass distribution by anatomical region.

Run from anywhere with:

    python scripts/compare_mass_distribution.py
"""

from __future__ import annotations

from pathlib import Path

import mujoco


REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = REPO_ROOT / "musclemimic_models" / "model" / "body"
MODEL_XMLS = (
    MODEL_DIR / "myofullbody.xml",
    MODEL_DIR / "myofullbody_pre_mod.xml",
)
REGION_ORDER = (
    "Head",
    "Thorax",
    "Abdomen / lumbar",
    "Pelvis",
    "Right arm",
    "Left arm",
    "Right leg",
    "Left leg",
)


def require_body_id(model: mujoco.MjModel, name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise ValueError(f"Required body was not found: {name!r}")
    return body_id


def descendants(model: mujoco.MjModel, root_name: str) -> set[int]:
    """Return the root body and all bodies below it."""
    root_id = require_body_id(model, root_name)
    result = {root_id}
    for body_id in range(1, model.nbody):
        ancestor_id = body_id
        while ancestor_id != 0:
            if ancestor_id == root_id:
                result.add(body_id)
                break
            ancestor_id = int(model.body_parentid[ancestor_id])
    return result


def region_body_ids(model: mujoco.MjModel, pre_mod: bool) -> dict[str, set[int]]:
    """Assign every non-world model body to one anatomical region."""
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id): body_id
        for body_id in range(1, model.nbody)
    }

    regions = {
        "Head": descendants(model, "head_attach"),
        "Abdomen / lumbar": {
            names[name]
            for name in ("Abdomen", "lumbar1", "lumbar2", "lumbar3", "lumbar4", "lumbar5")
        },
        "Pelvis": {names[name] for name in ("Full Body", "sacrum", "pelvis")},
        "Right arm": descendants(model, "clavicle" if pre_mod else "clavicle_r"),
        "Left arm": descendants(model, "clavicle_l"),
        "Right leg": descendants(model, "femur_r"),
        "Left leg": descendants(model, "femur_l"),
    }

    assigned = set().union(*regions.values())
    # The remaining bodies are torso structures and attachment/root bodies.
    regions["Thorax"] = set(range(1, model.nbody)) - assigned
    return regions


def region_masses(model: mujoco.MjModel, pre_mod: bool) -> dict[str, float]:
    regions = region_body_ids(model, pre_mod)
    return {
        region: sum(float(model.body_mass[body_id]) for body_id in body_ids)
        for region, body_ids in regions.items()
    }


def print_comparison(
    current_masses: dict[str, float], pre_mod_masses: dict[str, float]
) -> None:
    current_total = sum(current_masses.values())
    pre_mod_total = sum(pre_mod_masses.values())

    print(
        f"{'Body region':<20} {'Current kg':>12} {'Current %':>12} "
        f"{'Pre-mod kg':>12} {'Pre-mod %':>12} {'Delta kg':>12}"
    )
    print("-" * 83)
    for region in REGION_ORDER:
        current_mass = current_masses[region]
        pre_mod_mass = pre_mod_masses[region]
        current_percent = 100.0 * current_mass / current_total
        pre_mod_percent = 100.0 * pre_mod_mass / pre_mod_total
        print(
            f"{region:<20} {current_mass:>12.6f} {current_percent:>11.3f}% "
            f"{pre_mod_mass:>12.6f} {pre_mod_percent:>11.3f}% "
            f"{current_mass - pre_mod_mass:>12.6f}"
        )

    print("-" * 83)
    print(
        f"{'TOTAL':<20} {current_total:>12.6f} {'100.000%':>12} "
        f"{pre_mod_total:>12.6f} {'100.000%':>12} "
        f"{current_total - pre_mod_total:>12.6f}"
    )
    print("\nDelta kg = current - pre-mod.")


def main() -> None:
    current_model = mujoco.MjModel.from_xml_path(str(MODEL_XMLS[0]))
    pre_mod_model = mujoco.MjModel.from_xml_path(str(MODEL_XMLS[1]))
    current_masses = region_masses(current_model, pre_mod=False)
    pre_mod_masses = region_masses(pre_mod_model, pre_mod=True)

    print("Full-body mass distribution by anatomical region")
    print(f"Current XML: {MODEL_XMLS[0]}")
    print(f"Pre-mod XML: {MODEL_XMLS[1]}\n")
    print_comparison(current_masses, pre_mod_masses)


if __name__ == "__main__":
    main()
