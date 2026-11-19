"""Compare one-to-one MyoFullBody and MS-Human-700 muscle curves.

Neck muscles and anatomical mappings that are not one-to-one are deliberately
excluded. The script writes mapping CSVs and groups matching left/right
muscle-joint curves into two-panel bilateral plots so model differences and
within-model symmetry can be reviewed together. Unpaired matches fall back to
unilateral plots.

Run from anywhere with:

    python scripts/compare_myofullbody_mshuman700.py
    python scripts/compare_myofullbody_mshuman700.py --muscle soleus_r
"""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import scipy.io


FONT_SIZE = 14

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "STIXGeneral", "DejaVu Serif"],
    "mathtext.fontset": "stix",

    "font.size": FONT_SIZE,
    "axes.labelsize": FONT_SIZE,
    "axes.titlesize": FONT_SIZE,
    "xtick.labelsize": FONT_SIZE - 1,
    "ytick.labelsize": FONT_SIZE - 1,
    "legend.fontsize": FONT_SIZE - 1,

    "axes.labelcolor": "#222222",
    "text.color": "#222222",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

REPO_ROOT = Path(__file__).resolve().parents[1]
MYO_XML = REPO_ROOT / "musclemimic_models/model/body/myofullbody.xml"
PRE_MOD_XML = REPO_ROOT / "musclemimic_models/model/body/myofullbody_pre_mod.xml"
MS_XML = REPO_ROOT / "MS-Human-700/MS-Human-700.xml"
EXPERIMENT_DIR = REPO_ROOT / "scripts/experimental_validation/data"

SHOULDER_EXPERIMENT_MAP = {
    "teres_major": "TMAJ",
    "anterior_supraspinatus": "SUPSP",
    "posterior_supraspinatus": "SUPSP",
    "superior_subscapularis": "SUBSC",
    "middle_subscapularis": "SUBSC",
    "inferior_subscapularis": "SUBSC",
    "superior_infraspinatus": "INFSP",
    "inferior_infraspinatus": "INFSP",
    "teres_minor": "TMIN",
    "anterior_deltoid": "DELT1",
    "middle_deltoid": "DELT2",
    "posterior_deltoid": "DELT3",
    "superior_lat_dorsi": "LAT1",
    "middle_lat_dorsi": "LAT2",
    "inferior_lat_dorsi": "LAT3",
    "superior_pec_major": "PECM1",
    "middle_pec_major": "PECM2",
    "inferior_pec_major": "PECM3",
}

LOWER_EXPERIMENT_MAP = {
    "bflh": ("Biceps femoris (long)", "Knee", "knee_angle"),
    "bfsh": ("Biceps femoris (short)", "Knee", "knee_angle"),
    "edl": ("Extensor digitorum longus", "Ankle", "ankle_angle"),
    "ehl": ("Extensor hallucis longus", "Ankle", "ankle_angle"),
    "fdl": ("Flexor digitorum longus", "Ankle", "ankle_angle"),
    "fhl": ("Flexor hallucis longus", "Ankle", "ankle_angle"),
    "gaslat": ("Gastrocnemius lateralis", "Knee", "knee_angle"),
    "gasmed": ("Gastrocnemius medialis", "Knee", "knee_angle"),
    "grac": ("Gracilis", "Knee", "knee_angle"),
    "piri": ("Piriformis", "Hip", "hip_flexion"),
    "psoas": ("Psoas", "Hip", "hip_flexion"),
    "recfem": ("Rectus femoris", "Knee", "knee_angle"),
    "sart": ("Sartorius", "Knee", "knee_angle"),
    "semimem": ("Semimembranosus", "Knee", "knee_angle"),
    "semiten": ("Semitendinosus", "Knee", "knee_angle"),
    "soleus": ("Soleus", "Ankle", "ankle_angle"),
    "tfl": ("Tensor fasciae latae", "Hip", "hip_flexion"),
    "tibant": ("Tibialis anterior", "Ankle", "ankle_angle"),
    "tibpost": ("Tibialis posterior", "Ankle", "ankle_angle"),
    "vasint": ("Vastus intermedius", "Knee", "knee_angle"),
    "vaslat": ("Vastus lateralis", "Knee", "knee_angle"),
    "vasmed": ("Vastus medialis", "Knee", "knee_angle"),
}
for prefix, dataset_name in (("glmax", "Gluteus maximus"), ("glmed", "Gluteus medius")):
    for compartment in (1, 2, 3):
        LOWER_EXPERIMENT_MAP[f"{prefix}{compartment}"] = (
            dataset_name, "Hip", "hip_flexion"
        )

# These are spelling/side-label aliases only. No muscles are merged or split.
ARM_DIGIT_ALIASES = {
    "FDS5": "FDSL", "FDS4": "FDSR", "FDS3": "FDSM", "FDS2": "FDSI",
    "FDP5": "FDPL", "FDP4": "FDPR", "FDP3": "FDPM", "FDP2": "FDPI",
    "EDC5": "EDCL", "EDC4": "EDCR", "EDC3": "EDCM", "EDC2": "EDCI",
}

# Only coordinate renames with the same mechanical meaning are included.
JOINT_ALIASES = {
    "elbow_flex_r": "elbow_flexion_r",
    "elbow_flex_l": "elbow_flexion_l",
}
PRE_MOD_JOINT_ALIASES = {
    "elbow_flex_r": "elbow_flexion",
    "elbow_flex_l": "elbow_flexion_l",
}

# MS-Human-700 has neck actuators that MyoFullBody does not. Keep the exclusion
# explicit so future name additions cannot accidentally bring them into scope.
NECK_PREFIXES = (
    "stern_mast", "cleid_mast", "cleid_occ", "scalenus_", "long_col_",
    "splen_", "semi_cap_", "semi_cerv_", "longissi_cerv_", "iliocost_cerv_",
    "supmult-", "deepmult-",
)
for side in "rl":
    for coordinate in (
        "translation2", "translation1", "rotation2", "rotation3",
        "beta_translation2", "beta_translation1", "beta_rotation1",
    ):
        JOINT_ALIASES[f"knee_angle_{coordinate}_{side}"] = (
            f"knee_angle_{side}_{coordinate}"
        )


def names(model: mujoco.MjModel, obj_type, count: int) -> list[str]:
    return [
        name
        for index in range(count)
        if (name := mujoco.mj_id2name(model, obj_type, index)) is not None
    ]


def muscle_aliases(name: str) -> list[str]:
    """Return conservative aliases in preference order."""
    candidates = [name, name.replace(".", "_")]
    if name.endswith("_left"):
        stem, side = name[:-5], "_l"
    elif name.endswith(("_r", "_l")):
        return candidates
    else:
        stem, side = name, "_r"
    candidates.extend((stem + side, ARM_DIGIT_ALIASES.get(stem, stem) + side))
    return list(dict.fromkeys(candidates))


def build_mapping(source: list[str], targets: set[str]) -> tuple[dict[str, str], dict[str, str]]:
    mapped: dict[str, str] = {}
    deferred: dict[str, str] = {}
    claimed: dict[str, str] = {}
    for source_name in source:
        matches = [candidate for candidate in muscle_aliases(source_name) if candidate in targets]
        matches = list(dict.fromkeys(matches))
        if len(matches) != 1:
            deferred[source_name] = "no one-to-one name match" if not matches else "ambiguous name match"
            continue
        target = matches[0]
        if target in claimed:
            deferred[source_name] = f"target already mapped from {claimed[target]}"
            continue
        mapped[source_name] = target
        claimed[target] = source_name
    return mapped, deferred


def is_neck_muscle(name: str) -> bool:
    return name.startswith(NECK_PREFIXES)


def object_id(model: mujoco.MjModel, obj_type, name: str) -> int:
    result = mujoco.mj_name2id(model, obj_type, name)
    if result < 0:
        raise ValueError(f"Missing model object: {name}")
    return result


def equality_dependent_joints(model: mujoco.MjModel) -> set[str]:
    """Return joint1 coordinates controlled by a MuJoCo joint equality."""
    dependent = set()
    for equality_id in range(model.neq):
        if model.eq_type[equality_id] != mujoco.mjtEq.mjEQ_JOINT:
            continue
        joint_id = int(model.eq_obj1id[equality_id])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name is not None:
            dependent.add(name)
    return dependent


def joint_equality_map(
    model: mujoco.MjModel,
) -> dict[int, tuple[int, np.ndarray]]:
    """Map each active slave joint to its master and polynomial coefficients."""
    equality_map = {}
    for equality_id in range(model.neq):
        if model.eq_type[equality_id] != mujoco.mjtEq.mjEQ_JOINT:
            continue
        if hasattr(model, "eq_active0") and not model.eq_active0[equality_id]:
            continue
        slave_id = int(model.eq_obj1id[equality_id])
        master_id = int(model.eq_obj2id[equality_id])
        if slave_id < 0 or master_id < 0:
            continue
        equality_map[slave_id] = (
            master_id,
            np.asarray(model.eq_data[equality_id, :5], dtype=float).copy(),
        )
    return equality_map


def apply_joint_equalities(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    equality_map: dict[int, tuple[int, np.ndarray]],
) -> None:
    """Set dependent qpos values from MuJoCo joint equality polynomials."""
    for slave_id, (master_id, coefficients) in equality_map.items():
        slave_adr = int(model.jnt_qposadr[slave_id])
        master_adr = int(model.jnt_qposadr[master_id])
        master_value = float(data.qpos[master_adr])
        data.qpos[slave_adr] = sum(
            coefficient * master_value**power
            for power, coefficient in enumerate(coefficients)
        )


def curve(
    model: mujoco.MjModel,
    muscle: str,
    joint: str,
    samples: int,
    eps: float,
    activation: float,
) -> dict[str, np.ndarray]:
    actuator = object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle)
    joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    tendon = int(model.actuator_trnid[actuator, 0])
    qpos_adr = int(model.jnt_qposadr[joint_id])
    angles = np.linspace(*model.jnt_range[joint_id], samples)
    moment_arms = np.empty(samples)
    lengths = np.empty(samples)
    forces = np.empty(samples)
    data = mujoco.MjData(model)
    equality_map = joint_equality_map(model)

    for index, angle in enumerate(angles):
        data.qpos[:] = model.qpos0
        data.qpos[qpos_adr] = angle - eps
        apply_joint_equalities(model, data, equality_map)
        mujoco.mj_forward(model, data)
        before = data.ten_length[tendon]

        data.qpos[:] = model.qpos0
        data.qpos[qpos_adr] = angle + eps
        apply_joint_equalities(model, data, equality_map)
        mujoco.mj_forward(model, data)
        moment_arms[index] = -(data.ten_length[tendon] - before) / (2 * eps)

        data.qpos[:] = model.qpos0
        data.qvel[:] = 0
        data.ctrl[:] = 0
        data.qpos[qpos_adr] = angle
        apply_joint_equalities(model, data, equality_map)
        data.ctrl[actuator] = activation
        if model.na:
            data.act[:] = 0
            act_adr = model.actuator_actadr[actuator]
            if act_adr >= 0:
                data.act[act_adr] = activation
        mujoco.mj_forward(model, data)
        lengths[index] = data.actuator_length[actuator]
        forces[index] = -data.actuator_force[actuator]
    return {"angles": angles, "moment_arms": moment_arms, "lengths": lengths, "forces": forces}


def curves_for_joint(
    model: mujoco.MjModel,
    joint: str,
    samples: int,
    eps: float,
    activation: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Sample all actuators and tendons while sweeping one joint."""
    joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    qpos_adr = int(model.jnt_qposadr[joint_id])
    angles = np.linspace(*model.jnt_range[joint_id], samples)
    moment_arms = np.empty((samples, model.ntendon))
    lengths = np.empty((samples, model.nu))
    forces = np.empty((samples, model.nu))
    data = mujoco.MjData(model)
    equality_map = joint_equality_map(model)

    for index, angle in enumerate(angles):
        data.qpos[:] = model.qpos0
        data.qpos[qpos_adr] = angle - eps
        apply_joint_equalities(model, data, equality_map)
        mujoco.mj_forward(model, data)
        before = data.ten_length.copy()

        data.qpos[:] = model.qpos0
        data.qpos[qpos_adr] = angle + eps
        apply_joint_equalities(model, data, equality_map)
        mujoco.mj_forward(model, data)
        moment_arms[index] = -(data.ten_length - before) / (2.0 * eps)

        data.qpos[:] = model.qpos0
        data.qvel[:] = 0.0
        data.qpos[qpos_adr] = angle
        apply_joint_equalities(model, data, equality_map)
        data.ctrl[:] = activation
        if model.na:
            data.act[:] = activation
        mujoco.mj_forward(model, data)
        lengths[index] = data.actuator_length
        forces[index] = -data.actuator_force

    return angles, moment_arms, lengths, forces


def select_curve(
    sampled: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
    actuator_id: int,
    tendon_id: int,
) -> dict[str, np.ndarray]:
    angles, moment_arms, lengths, forces = sampled
    return {
        "angles": angles,
        "moment_arms": moment_arms[:, tendon_id],
        "lengths": lengths[:, actuator_id],
        "forces": forces[:, actuator_id],
    }


def spans_joint(
    model: mujoco.MjModel, muscle: str, joint: str, eps: float, tolerance: float,
) -> bool:
    actuator = object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle)
    tendon = int(model.actuator_trnid[actuator, 0])
    joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    qpos_adr = int(model.jnt_qposadr[joint_id])
    data = mujoco.MjData(model)
    equality_map = joint_equality_map(model)
    lower, upper = model.jnt_range[joint_id]
    for angle in (lower, 0.5 * (lower + upper), upper):
        data.qpos[:] = model.qpos0
        data.qpos[qpos_adr] = angle - eps
        apply_joint_equalities(model, data, equality_map)
        mujoco.mj_forward(model, data)
        before = data.ten_length[tendon]

        data.qpos[:] = model.qpos0
        data.qpos[qpos_adr] = angle + eps
        apply_joint_equalities(model, data, equality_map)
        mujoco.mj_forward(model, data)
        if abs((data.ten_length[tendon] - before) / (2 * eps)) > tolerance:
            return True
    return False


def safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value)


def remove_numbered_conflict_copies(output_dir: Path) -> int:
    """Remove Finder/cloud conflict copies when the canonical export exists."""
    removed = 0
    pattern = re.compile(r"^(.*) [0-9]+(\.(?:png|svg|csv))$")
    for path in output_dir.iterdir():
        if not path.is_file() or (match := pattern.match(path.name)) is None:
            continue
        canonical = output_dir / f"{match.group(1)}{match.group(2)}"
        if canonical.exists():
            path.unlink()
            removed += 1
    return removed


def first_existing(candidates: list[str], available: set[str]) -> str | None:
    return next((candidate for candidate in candidates if candidate in available), None)


def pre_mod_muscle_name(current_name: str, available: set[str]) -> str | None:
    candidates = [current_name]
    if current_name.endswith("_left"):
        candidates.append(current_name[:-5] + "_l")
    elif current_name.endswith("_r"):
        candidates.append(current_name[:-2])
    return first_existing(candidates, available)


def pre_mod_joint_name(
    current_name: str, ms_name: str, available: set[str]
) -> str | None:
    candidates = [current_name]
    if current_name in PRE_MOD_JOINT_ALIASES:
        candidates.append(PRE_MOD_JOINT_ALIASES[current_name])
    if current_name.endswith("_r"):
        candidates.append(current_name[:-2])
    candidates.append(ms_name)
    return first_existing(list(dict.fromkeys(candidates)), available)


def side_and_base(name: str) -> tuple[str, str | None]:
    if name.endswith("_r"):
        return name[:-2], "right"
    if name.endswith("_l"):
        return name[:-2], "left"
    return name, None


def _experimental_curve(
    angle_deg: np.ndarray, moment_arm_mm: np.ndarray, source: str
) -> dict[str, np.ndarray | str] | None:
    count = min(len(angle_deg), len(moment_arm_mm))
    angle_deg = np.asarray(angle_deg[:count], dtype=float)
    moment_arm_mm = np.asarray(moment_arm_mm[:count], dtype=float)
    valid = np.isfinite(angle_deg) & np.isfinite(moment_arm_mm)
    if np.count_nonzero(valid) < 2:
        return None
    return {
        "angles": np.deg2rad(angle_deg[valid]),
        "moment_arms": moment_arm_mm[valid] / 1000.0,
        "source": source,
    }


def _mat_numeric_blocks(value: object) -> list[tuple[np.ndarray, np.ndarray]]:
    if isinstance(value, np.ndarray) and value.ndim == 2:
        containers = [value]
    elif isinstance(value, np.ndarray) and value.ndim == 1:
        containers = list(value)
    elif isinstance(value, (list, tuple)):
        containers = list(value)
    else:
        containers = [value]
    extracted = []
    for block in containers:
        if block is None or np.isscalar(block):
            continue
        for subblock in block if isinstance(block, (list, tuple)) else [block]:
            array = np.asarray(subblock)
            if array.ndim != 2:
                continue
            if array.shape[0] == 3:
                extracted.append((array[0, :], array[2, :]))
            elif array.shape[1] == 3:
                extracted.append((array[:, 0], array[:, 2]))
    return extracted


def load_experimental_curves() -> dict[
    tuple[str, str], list[dict[str, np.ndarray | str]]
]:
    """Load only published measurements with explicit muscle/joint matches."""
    result: dict[tuple[str, str], list[dict[str, np.ndarray | str]]] = {}

    pigeon_path = EXPERIMENT_DIR / "pigeon_datasets.csv"
    if pigeon_path.exists():
        with pigeon_path.open(newline="") as handle:
            rows = list(csv.reader(handle))
        for column in range(0, len(rows[0]), 2):
            muscle = rows[0][column].strip()
            if not muscle:
                continue
            x, y = [], []
            for row in rows[2:]:
                try:
                    x.append(float(row[column]))
                    y.append(float(row[column + 1]))
                except (ValueError, IndexError):
                    continue
            curve_data = _experimental_curve(
                np.asarray(x), np.asarray(y), "Pigeon et al. (1996)"
            )
            if curve_data is not None:
                result.setdefault((muscle, "elbow_flexion"), []).append(curve_data)

    shoulder_path = EXPERIMENT_DIR / "shoulder_dataset_Ackland.csv"
    if shoulder_path.exists():
        grouped: dict[tuple[str, str], tuple[list[float], list[float]]] = {}
        with shoulder_path.open(newline="") as handle:
            for row in csv.DictReader(handle):
                if row["joint_action"] != "abduction":
                    continue
                base = SHOULDER_EXPERIMENT_MAP.get(row["region"])
                if base is None:
                    continue
                angles, moments = grouped.setdefault((base, row["region"]), ([], []))
                angles.append(float(row["angle_deg"]))
                moments.append(float(row["moment_arm_mm"]))
        for (base, _region), (angles, moments) in grouped.items():
            curve_data = _experimental_curve(
                np.asarray(angles), np.asarray(moments), "Ackland et al. (2008)"
            )
            if curve_data is not None:
                result.setdefault((base, "shoulder_elv"), []).append(curve_data)

    lower_path = EXPERIMENT_DIR / "S3_Datasets.mat"
    if lower_path.exists():
        dataset = scipy.io.loadmat(
            lower_path, squeeze_me=True, struct_as_record=False
        )["dataset_momentArm"]
        for base, (muscle, joint, joint_base) in LOWER_EXPERIMENT_MAP.items():
            for entry in dataset:
                if (
                    str(entry.measureObject) != muscle
                    or str(entry.primaryDoF).split()[0] != joint
                ):
                    continue
                for angles, moments in _mat_numeric_blocks(entry.measureValue):
                    curve_data = _experimental_curve(
                        np.asarray(angles), np.asarray(moments), str(entry.reference)
                    )
                    if curve_data is not None:
                        result.setdefault((base, joint_base), []).append(curve_data)
    return result


def save_group_plot(
    side_curves: dict[str, dict[str, dict[str, np.ndarray]]],
    muscle: str,
    joint: str,
    activation: float,
    output: Path,
    experimental: list[dict[str, np.ndarray | str]] | None = None,
) -> None:
    # 7.16 in is the typical usable width of a two-column IEEE/robotics paper.
    experimental_source_count = len({
        str(values["source"]) for values in (experimental or [])
    })
    # Keep the axes at a fixed physical height in every output. Extra
    # experimental-reference rows enlarge only the lower legend band.
    axes_height_in = 1.15
    top_margin_in = 0.15
    if experimental_source_count > 2:
        figure_height_in = 3.4
        model_legend_y_in = 0.90
    elif experimental_source_count:
        figure_height_in = 3.15
        model_legend_y_in = 0.68
    else:
        figure_height_in = 2.90
        model_legend_y_in = 0.35
    bottom_margin_in = figure_height_in - top_margin_in - axes_height_in
    fig, axes = plt.subplots(
        1, 2, figsize=(7.36, figure_height_in), facecolor="white"
    )
    # Muted, colorblind-safe model colors. Side identity is encoded by a
    # consistent solid/dashed pattern, width, opacity, and plotting order.
    colors = {
        "MyoFullBody": "#2166AC",
        "MS-Human-700": "#D6604D",
        "myo_sim": "#1B9E77",
    }
    # Models are drawn in this order. Give earlier curves a wider stroke so a
    # coincident curve plotted later cannot hide them completely.
    model_width_scale = {
        "MyoFullBody": 1.45,
        "MS-Human-700": 1.20,
        "myo_sim": 1.00,
    }
    side_styles = {
        "right": {
            "linestyle": "-",
            "linewidth": 2.2,
            "alpha": 0.62,
            "zorder": 2,
        },
        "left": {
            "linestyle": (0, (5, 2.2)),
            "linewidth": 1.55,
            "alpha": 1.0,
            "zorder": 3,
        },
        "unilateral": {
            "linestyle": "-",
            "linewidth": 1.8,
            "alpha": 1.0,
            "zorder": 2,
        },
    }
    for side in ("right", "left", "unilateral"):
        if side not in side_curves:
            continue
        for model_name in ("MyoFullBody", "MS-Human-700", "myo_sim"):
            if model_name not in side_curves[side]:
                continue
            values = side_curves[side][model_name]
            side_label = {"right": "R", "left": "L"}.get(side)
            label = model_name if side_label is None else f"{model_name} {side_label}"
            side_style = dict(side_styles[side])
            side_style["linewidth"] *= model_width_scale[model_name]
            style = {
                "color": colors[model_name],
                "label": label,
                **side_style,
            }
            axes[0].plot(values["angles"], values["moment_arms"], **style)
            axes[1].plot(values["lengths"], values["forces"], **style)
    labelled_sources: set[str] = set()
    for values in experimental or []:
        source = str(values["source"])
        label = (
            f"Experimental data — {source}"
            if source not in labelled_sources
            else "_nolegend_"
        )
        labelled_sources.add(source)
        axes[0].plot(
            values["angles"],
            values["moment_arms"],
            color="#262626",
            linestyle=(0, (1.2, 1.8)),
            linewidth=1.0,
            marker="o",
            markersize=2.2,
            markerfacecolor="white",
            markeredgewidth=0.65,
            alpha=0.72,
            zorder=4,
            label=label,
        )
    joint_label = joint.replace("_", " ").capitalize()
    muscle_label = muscle.replace("_", " ")
    joint_axis_label = (
        f"{joint_label} (rad)"
        if joint_label.lower().endswith("angle")
        else f"{joint_label} angle (rad)"
    )
    axes[0].set(
        xlabel=joint_axis_label,
        ylabel=f"{muscle_label}\nMoment arm (m)",
    )
    axes[1].set(
        xlabel=f"{muscle_label} MTU length (m)",
        ylabel="Muscle force (N)",
    )
    for axis in axes:
        axis.set_facecolor("white")
        axis.set_axisbelow(True)
        axis.grid(True, color="#D9DEE3", linewidth=0.45, alpha=0.6)
        axis.tick_params(
            axis="both", width=0.8, length=3.5,
            color="#4A4A4A", labelcolor="#222222",
        )
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
        axis.spines["left"].set_color("#4A4A4A")
        axis.spines["bottom"].set_color("#4A4A4A")
        axis.spines["left"].set_linewidth(0.75)
        axis.spines["bottom"].set_linewidth(0.75)
    handles, labels = axes[0].get_legend_handles_labels()
    experimental_items = [
        (handle, label)
        for handle, label in zip(handles, labels)
        if label.startswith("Experimental data")
    ]
    model_items = [
        (handle, label)
        for handle, label in zip(handles, labels)
        if not label.startswith("Experimental data")
    ]
    model_item_by_label = {label: (handle, label) for handle, label in model_items}
    bilateral_order = [
        f"{model_name} {side}"
        for model_name in ("MyoFullBody", "MS-Human-700", "myo_sim")
        for side in ("R", "L")
    ]
    if all(label in model_item_by_label for label in bilateral_order):
        # Matplotlib fills each of the three columns vertically. Pair R/L for
        # each model so the displayed first row is R and the second row is L.
        model_items = [model_item_by_label[label] for label in bilateral_order]
    fig.legend(
        [item[0] for item in model_items],
        [item[1] for item in model_items],
        loc="lower center",
        bbox_to_anchor=(0.5, model_legend_y_in / figure_height_in),
        ncol=min(3, len(model_items)),
        frameon=False,
        handlelength=2.8,
        handletextpad=0.5,
        columnspacing=1.2,
    )
    if experimental_items:
        fig.legend(
            [item[0] for item in experimental_items],
            [item[1] for item in experimental_items],
            loc="lower center",
            bbox_to_anchor=(0.5, 0.03 / figure_height_in),
            ncol=min(2, len(experimental_items)),
            frameon=False,
            handlelength=2.4,
            handletextpad=0.5,
            columnspacing=1.1,
        )
    # Fixed margins keep both axes exactly the same physical width and height;
    # tight_layout can resize them differently when tick-label widths differ.
    fig.subplots_adjust(
        left=0.1634,
        right=0.9660,
        bottom=bottom_margin_in / figure_height_in,
        top=1.0 - top_margin_in / figure_height_in,
        wspace=0.34,
    )

    # Save to hidden temporary files and atomically replace the canonical
    # exports. This minimizes Finder/cloud conflict copies such as " 2.svg".
    svg_output = output.with_suffix(".svg")
    png_temp = output.with_name(f".{output.stem}.tmp.png")
    svg_temp = output.with_name(f".{output.stem}.tmp.svg")
    for temporary in (png_temp, svg_temp):
        temporary.unlink(missing_ok=True)
    fig.savefig(png_temp, dpi=300, facecolor="white")
    fig.savefig(svg_temp, facecolor="white")
    png_temp.replace(output)
    svg_temp.replace(svg_output)
    plt.close(fig)


def write_rows(path: Path, header: list[str], rows: list[list[object]]) -> None:
    temporary = path.with_name(f".{path.stem}.tmp{path.suffix}")
    temporary.unlink(missing_ok=True)
    with temporary.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)
    temporary.replace(path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--eps", type=float, default=1e-5)
    parser.add_argument("--activation", type=float, default=1.0)
    parser.add_argument("--influence-tol", type=float, default=1e-6)
    parser.add_argument("--muscle", action="append", help="MyoFullBody muscle; repeat as needed")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "scripts/output/myofullbody_vs_mshuman700",
        help="Plot directory (default: scripts/output/myofullbody_vs_mshuman700)",
    )
    args = parser.parse_args()
    if args.samples < 2:
        parser.error("--samples must be at least 2")
    if not 0 <= args.activation <= 1:
        parser.error("--activation must be between 0 and 1")

    print("Loading MyoFullBody...", flush=True)
    myo_model = mujoco.MjModel.from_xml_path(str(MYO_XML))
    print("Loading myo_sim...", flush=True)
    pre_mod_model = mujoco.MjModel.from_xml_path(str(PRE_MOD_XML))
    print("Loading MS-Human-700...", flush=True)
    ms_model = mujoco.MjModel.from_xml_path(str(MS_XML))
    myo_muscles = names(myo_model, mujoco.mjtObj.mjOBJ_ACTUATOR, myo_model.nu)
    ms_muscles = {
        name for name in names(ms_model, mujoco.mjtObj.mjOBJ_ACTUATOR, ms_model.nu)
        if not is_neck_muscle(name)
    }
    muscle_map, deferred = build_mapping(myo_muscles, ms_muscles)
    if args.muscle:
        missing = set(args.muscle) - muscle_map.keys()
        if missing:
            parser.error(f"Requested muscles have no one-to-one mapping: {sorted(missing)}")
        requested_bases = {
            side_and_base(muscle_map[name])[0] for name in args.muscle
        }
        muscle_map = {
            name: target
            for name, target in muscle_map.items()
            if side_and_base(target)[0] in requested_bases
        }

    myo_joints = names(myo_model, mujoco.mjtObj.mjOBJ_JOINT, myo_model.njnt)
    pre_mod_muscles = set(
        names(pre_mod_model, mujoco.mjtObj.mjOBJ_ACTUATOR, pre_mod_model.nu)
    )
    pre_mod_joints = set(
        names(pre_mod_model, mujoco.mjtObj.mjOBJ_JOINT, pre_mod_model.njnt)
    )
    ms_joints = set(names(ms_model, mujoco.mjtObj.mjOBJ_JOINT, ms_model.njnt))
    myo_dependent_joints = equality_dependent_joints(myo_model)
    ms_dependent_joints = equality_dependent_joints(ms_model)
    joint_map = {
        name: target for name in myo_joints
        if (target := JOINT_ALIASES.get(name, name)) in ms_joints
        and model_joint_is_limited(myo_model, name) and model_joint_is_limited(ms_model, target)
        and name not in myo_dependent_joints
        and target not in ms_dependent_joints
    }

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stale_plots = [
        path
        for suffix in ("*.png", "*.svg")
        for path in output_dir.glob(f"*__*__{suffix}")
    ]
    for path in stale_plots:
        path.unlink()
    if stale_plots:
        print(f"Removed {len(stale_plots)} plots from the previous run.", flush=True)
    for temporary in output_dir.glob(".*.tmp.*"):
        temporary.unlink()
    mapping_rows = [[name, target, "mapped"] for name, target in muscle_map.items()]
    mapping_rows += [[name, "", reason] for name, reason in deferred.items()]
    write_rows(output_dir / "muscle_mapping.csv", ["myofullbody", "mshuman700", "status"], mapping_rows)
    write_rows(output_dir / "joint_mapping.csv", ["myofullbody", "mshuman700"], [[a, b] for a, b in joint_map.items()])

    pairs = 0
    grouped_curves: dict[
        tuple[str, str], dict[str, dict[str, dict[str, np.ndarray]]]
    ] = {}

    # Resolve IDs once. The old implementation swept a joint separately for
    # every muscle, which made a full run look stalled for several minutes.
    muscle_info = []
    for muscle, ms_muscle in muscle_map.items():
        myo_actuator = object_id(
            myo_model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle
        )
        ms_actuator = object_id(
            ms_model, mujoco.mjtObj.mjOBJ_ACTUATOR, ms_muscle
        )
        pre_muscle = pre_mod_muscle_name(muscle, pre_mod_muscles)
        pre_actuator = (
            object_id(pre_mod_model, mujoco.mjtObj.mjOBJ_ACTUATOR, pre_muscle)
            if pre_muscle is not None else None
        )
        muscle_info.append({
            "muscle": muscle,
            "ms_muscle": ms_muscle,
            "myo_actuator": myo_actuator,
            "myo_tendon": int(myo_model.actuator_trnid[myo_actuator, 0]),
            "ms_actuator": ms_actuator,
            "ms_tendon": int(ms_model.actuator_trnid[ms_actuator, 0]),
            "pre_actuator": pre_actuator,
            "pre_tendon": (
                int(pre_mod_model.actuator_trnid[pre_actuator, 0])
                if pre_actuator is not None else None
            ),
        })

    total_joints = len(joint_map)
    for joint_index, (joint, ms_joint) in enumerate(joint_map.items(), start=1):
        print(
            f"Scanning joint {joint_index}/{total_joints}: {joint} / {ms_joint}",
            flush=True,
        )
        myo_sampled = curves_for_joint(
            myo_model, joint, args.samples, args.eps, args.activation
        )
        ms_sampled = curves_for_joint(
            ms_model, ms_joint, args.samples, args.eps, args.activation
        )
        pre_joint = pre_mod_joint_name(joint, ms_joint, pre_mod_joints)
        pre_sampled = (
            curves_for_joint(
                pre_mod_model, pre_joint, args.samples, args.eps, args.activation
            )
            if pre_joint is not None else None
        )

        for info in muscle_info:
            muscle = info["muscle"]
            ms_muscle = info["ms_muscle"]
            myo = select_curve(
                myo_sampled, info["myo_actuator"], info["myo_tendon"]
            )
            ms = select_curve(
                ms_sampled, info["ms_actuator"], info["ms_tendon"]
            )
            if (
                np.max(np.abs(myo["moment_arms"])) <= args.influence_tol
                and np.max(np.abs(ms["moment_arms"])) <= args.influence_tol
            ):
                continue
            muscle_base, muscle_side = side_and_base(ms_muscle)
            joint_base, joint_side = side_and_base(ms_joint)
            if muscle_side is not None and joint_side is not None and muscle_side != joint_side:
                continue
            side = muscle_side or joint_side or "unilateral"
            grouped_curves.setdefault((muscle_base, joint_base), {})[side] = {
                "MyoFullBody": myo,
                "MS-Human-700": ms,
            }
            if pre_sampled is not None and info["pre_actuator"] is not None:
                grouped_curves[(muscle_base, joint_base)][side]["myo_sim"] = select_curve(
                    pre_sampled,
                    info["pre_actuator"],
                    info["pre_tendon"],
                )
            pairs += 1

    print("Loading matching experimental moment-arm data...", flush=True)
    experimental_curves = load_experimental_curves()
    matched_experimental_plots = sum(
        key in experimental_curves for key in grouped_curves
    )
    print(
        f"Experimental overlays available for {matched_experimental_plots} plots.",
        flush=True,
    )

    plots = bilateral = 0
    for (muscle_base, joint_base), side_curves in sorted(grouped_curves.items()):
        has_both_sides = "right" in side_curves and "left" in side_curves
        suffix = "bilateral" if has_both_sides else next(iter(side_curves))
        output = output_dir / safe_name(
            f"{muscle_base}__{joint_base}__{suffix}.png"
        )
        save_group_plot(
            side_curves,
            muscle_base,
            joint_base,
            args.activation,
            output,
            experimental_curves.get((muscle_base, joint_base)),
        )
        plots += 1
        bilateral += int(has_both_sides)
        print(
            f"saved: {muscle_base} @ {joint_base} [{suffix}] -> {output.name}",
            flush=True,
        )

    print("\nComparison complete")
    print(f"One-to-one muscles : {len(muscle_map)}")
    print(f"Deferred muscles   : {len(deferred)}")
    print(f"Mapped joints      : {len(joint_map)}")
    print(f"Spanning pairs     : {pairs}")
    print(f"Plots saved        : {plots}")
    print(f"Bilateral plots    : {bilateral}")
    print(f"Experimental plots : {matched_experimental_plots}")
    print(f"Output directory   : {output_dir}")
    conflict_copies = remove_numbered_conflict_copies(output_dir)
    if conflict_copies:
        print(f"Conflict copies removed: {conflict_copies}")


def model_joint_is_limited(model: mujoco.MjModel, name: str) -> bool:
    joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return bool(model.jnt_limited[joint_id]) and model.jnt_range[joint_id, 0] < model.jnt_range[joint_id, 1]


if __name__ == "__main__":
    main()
