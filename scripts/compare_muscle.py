"""Compare current versus pre-mod muscles for a selected body region.

For every muscle/joint pair spanned by either model, this script compares the
moment-arm and force-length curves. It saves a two-panel plot only when a pair
has a discrepancy.

Run from anywhere with:

    uv run python scripts/compare_muscle.py --model bimanual
    uv run python scripts/compare_muscle.py --model torso
    uv run python scripts/compare_muscle.py --model legs
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = REPO_ROOT / "musclemimic_models" / "model"
MODEL_XMLS = {
    "bimanual": (
        MODEL_DIR / "arm" / "myoarm_bimanual.xml",
        MODEL_DIR / "arm" / "myoarm_bimanual_pre_mod.xml",
    ),
    # Torso and legs are loaded from the full-body files, then filtered below.
    "torso": (
        MODEL_DIR / "body" / "myofullbody.xml",
        MODEL_DIR / "body" / "myofullbody_pre_mod.xml",
    ),
    "legs": (
        MODEL_DIR / "body" / "myofullbody.xml",
        MODEL_DIR / "body" / "myofullbody_pre_mod.xml",
    ),
}


def object_names(model: mujoco.MjModel, obj_type, count: int) -> list[str]:
    return [
        name
        for index in range(count)
        if (name := mujoco.mj_id2name(model, obj_type, index)) is not None
    ]


def require_id(model: mujoco.MjModel, obj_type, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, obj_type, name)
    if obj_id < 0:
        raise ValueError(f"Required model object was not found: {name!r}")
    return obj_id


def moment_arm_curve(
    model: mujoco.MjModel,
    tendon_id: int,
    joint_id: int,
    samples: int,
    eps: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return joint angle and -d(tendon length)/dq using central differences."""
    data = mujoco.MjData(model)
    qpos_adr = model.jnt_qposadr[joint_id]
    angles = np.linspace(*model.jnt_range[joint_id], samples)
    moment_arms = np.empty(samples)

    for index, angle in enumerate(angles):
        data.qpos[:] = model.qpos0
        data.qpos[qpos_adr] = angle - eps
        mujoco.mj_forward(model, data)
        length_before = data.ten_length[tendon_id]

        data.qpos[qpos_adr] = angle + eps
        mujoco.mj_forward(model, data)
        length_after = data.ten_length[tendon_id]
        moment_arms[index] = -(length_after - length_before) / (2.0 * eps)

    return angles, moment_arms


def has_moment_arm(
    model: mujoco.MjModel,
    tendon_id: int,
    joint_id: int,
    eps: float,
    influence_tol: float,
) -> bool:
    """Cheaply screen a pair at the range endpoints and midpoint."""
    data = mujoco.MjData(model)
    qpos_adr = model.jnt_qposadr[joint_id]
    q0, q1 = model.jnt_range[joint_id]
    for angle in (q0, 0.5 * (q0 + q1), q1):
        data.qpos[:] = model.qpos0
        data.qpos[qpos_adr] = angle - eps
        mujoco.mj_forward(model, data)
        length_before = data.ten_length[tendon_id]
        data.qpos[qpos_adr] = angle + eps
        mujoco.mj_forward(model, data)
        moment_arm = -(data.ten_length[tendon_id] - length_before) / (2.0 * eps)
        if abs(moment_arm) > influence_tol:
            return True
    return False


def force_length_curve(
    model: mujoco.MjModel,
    actuator_id: int,
    joint_id: int,
    samples: int,
    activation: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return MTU length and tensile muscle force in joint-angle sample order."""
    data = mujoco.MjData(model)
    qpos_adr = model.jnt_qposadr[joint_id]
    angles = np.linspace(*model.jnt_range[joint_id], samples)
    lengths = np.empty(samples)
    forces = np.empty(samples)
    act_adr = model.actuator_actadr[actuator_id]

    for index, angle in enumerate(angles):
        data.qpos[:] = model.qpos0
        data.qvel[:] = 0.0
        data.qpos[qpos_adr] = angle
        data.ctrl[:] = 0.0
        data.ctrl[actuator_id] = activation
        if model.na:
            data.act[:] = 0.0
            if act_adr >= 0:
                data.act[act_adr] = activation
        mujoco.mj_forward(model, data)
        lengths[index] = data.actuator_length[actuator_id]
        forces[index] = -data.actuator_force[actuator_id]

    return lengths, forces


def analyze_pair(
    model: mujoco.MjModel,
    muscle: str,
    joint: str,
    samples: int,
    eps: float,
    activation: float,
) -> dict:
    actuator_id = require_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle)
    joint_id = require_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    tendon_id = model.actuator_trnid[actuator_id, 0]
    if tendon_id < 0:
        raise ValueError(f"Actuator {muscle!r} is not tendon-driven")

    angles, moment_arms = moment_arm_curve(
        model, tendon_id, joint_id, samples, eps
    )
    lengths, forces = force_length_curve(
        model, actuator_id, joint_id, samples, activation
    )
    return {
        "angles": angles,
        "moment_arms": moment_arms,
        "lengths": lengths,
        "forces": forces,
    }


def max_percent_difference(current_values, pre_mod_values) -> float:
    """Return the largest symmetric percentage difference in two arrays."""
    scale = np.maximum(np.abs(current_values), np.abs(pre_mod_values))
    difference = np.abs(current_values - pre_mod_values)
    percentages = np.zeros_like(difference)
    np.divide(100.0 * difference, scale, out=percentages, where=scale != 0.0)
    return float(np.max(percentages))


def curve_differences(current: dict, pre_mod: dict) -> dict[str, float | bool]:
    return {
        "angle_ranges": not np.allclose(
            current["angles"], pre_mod["angles"], atol=1e-10
        ),
        "moment_percent": max_percent_difference(
            current["moment_arms"], pre_mod["moment_arms"]
        ),
        "length_percent": max_percent_difference(
            current["lengths"], pre_mod["lengths"]
        ),
        "force_absolute": float(
            np.max(np.abs(current["forces"] - pre_mod["forces"]))
        ),
    }


def curves_disagree(
    differences: dict[str, float | bool],
    percent_tol: float,
    force_tol: float,
) -> bool:
    return any(
        (
            differences["angle_ranges"],
            differences["moment_percent"] > percent_tol,
            differences["length_percent"] > percent_tol,
            differences["force_absolute"] > force_tol,
        )
    )


def safe_filename(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value)


def names_between(names: list[str], first: str, stop: str | None = None) -> set[str]:
    """Return an ordered model section delimited by known actuator names."""
    start = names.index(first)
    end = names.index(stop, start) if stop is not None else len(names)
    return set(names[start:end])


def region_muscles(model: mujoco.MjModel, region: str, pre_mod: bool) -> set[str]:
    names = object_names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
    if region == "bimanual":
        left_suffix = "_l" if pre_mod else "_left"
        return {name for name in names if not name.endswith(left_suffix)}
    if region == "torso":
        return names_between(names, names[0], "DELT1")
    return names_between(names, "addbrev_r")


def region_joint_pairs(
    current_model: mujoco.MjModel,
    pre_mod_model: mujoco.MjModel,
    region: str,
) -> list[tuple[str, str]]:
    current_names = object_names(
        current_model, mujoco.mjtObj.mjOBJ_JOINT, current_model.njnt
    )
    pre_mod_names = set(
        object_names(pre_mod_model, mujoco.mjtObj.mjOBJ_JOINT, pre_mod_model.njnt)
    )
    if region == "bimanual":
        return sorted(
            (name, name[:-2])
            for name in current_names
            if name.endswith("_r") and name[:-2] in pre_mod_names
        )
    if region == "torso":
        torso_names = current_names[
            current_names.index("flex_extension") : current_names.index(
                "sternoclavicular_r2_r"
            )
        ]
        return [(name, name) for name in torso_names if name in pre_mod_names]

    pairs = []
    leg_names = current_names[current_names.index("hip_flexion_r") :]
    for name in leg_names:
        pre_mod_name = name
        if name.startswith("knee_angle_") and name.endswith(("_r", "_l")):
            side = name[-1]
            pre_mod_name = f"knee_angle_{side}_" + name[len("knee_angle_") : -2]
        if pre_mod_name in pre_mod_names:
            pairs.append((name, pre_mod_name))
    return pairs


def save_plot(
    curves: dict[str, dict],
    muscle: str,
    joint: str,
    activation: float,
    output: Path,
    region: str,
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(10, 5.8))
    styles = {
        "Current": {
            "label": "Musclemimic",
            "linestyle": "-",
            "color": "#005A8D",
        },
        "Pre-mod": {
            "label": "myo_sim",
            "linestyle": "--",
            "color": "#A84400",
        },
    }
    for label, curve in curves.items():
        plot_style = styles[label]
        axes[0].plot(
            curve["angles"],
            curve["moment_arms"],
            linewidth=3.0,
            solid_capstyle="round",
            solid_joinstyle="round",
            zorder=3,
            **plot_style,
        )
        axes[1].plot(
            curve["lengths"],
            curve["forces"],
            linewidth=3.0,
            solid_capstyle="round",
            solid_joinstyle="round",
            zorder=3,
            **plot_style,
        )

    axes[0].set_title("Moment arm")
    axes[0].set_xlabel(f"{joint} angle (rad)")
    axes[0].set_ylabel(f"{muscle} Moment Arm (m)")
    axes[1].set_title(f"Force–length (a = {activation:g})")
    axes[1].set_xlabel(f"{muscle} MTU length (m)")
    axes[1].set_ylabel(f"Force (N)")
    for axis in axes:
        axis.set_axisbelow(True)
        axis.grid(True, color="#D9D9D9", linewidth=0.8, alpha=0.7)
        axis.tick_params(axis="both", labelsize=18, width=1.1)
        axis.xaxis.label.set_size(18)
        axis.yaxis.label.set_size(18)
        axis.title.set_size(18)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.01),
        ncol=len(labels),
        frameon=False,
        fontsize=18,
        handlelength=3.0,
        columnspacing=2.5,
    )
    #fig.suptitle(f"{region.title()}: {muscle} at {joint}", fontsize=16, y=0.98)
    fig.tight_layout(rect=(0, 0.12, 1, 0.94), w_pad=2.5)
    fig.savefig(output, format="svg", dpi=200, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        choices=tuple(MODEL_XMLS),
        default="bimanual",
        help="Body region to compare (default: bimanual)",
    )
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--eps", type=float, default=1e-5)
    parser.add_argument("--activation", type=float, default=1.0)
    parser.add_argument(
        "--muscle",
        action="append",
        help="Limit the scan to a muscle name; repeat for multiple muscles",
    )
    parser.add_argument(
        "--percent-tol",
        type=float,
        default=10.0,
        help="Relative tolerance for moment arm and MTU length, in percent (default: 1)",
    )
    parser.add_argument(
        "--force-tol",
        type=float,
        default=1.0,
        help="Final absolute force-difference limit in N (default: 1)",
    )
    parser.add_argument(
        "--influence-tol",
        type=float,
        default=1e-6,
        help="Skip pairs whose moment arms stay below this value in both models",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Plot directory (default: scripts/output/<model>_current_vs_pre_mod)",
    )
    args = parser.parse_args()

    if args.samples < 2:
        parser.error("--samples must be at least 2")
    if not 0.0 <= args.activation <= 1.0:
        parser.error("--activation must be between 0 and 1")
    if args.percent_tol < 0.0:
        parser.error("--percent-tol cannot be negative")
    if args.force_tol < 0.0:
        parser.error("--force-tol cannot be negative")

    current_xml, pre_mod_xml = MODEL_XMLS[args.model]
    current_model = mujoco.MjModel.from_xml_path(str(current_xml))
    pre_mod_model = mujoco.MjModel.from_xml_path(str(pre_mod_xml))

    current_muscles = region_muscles(current_model, args.model, pre_mod=False)
    pre_mod_muscles = region_muscles(pre_mod_model, args.model, pre_mod=True)
    muscles = sorted(current_muscles & pre_mod_muscles)
    if args.muscle:
        requested = set(args.muscle)
        missing = requested - set(muscles)
        if missing:
            parser.error(f"Muscles not common to both models: {sorted(missing)}")
        muscles = [name for name in muscles if name in requested]

    joint_pairs = region_joint_pairs(current_model, pre_mod_model, args.model)

    output_dir = (
        args.output_dir
        or REPO_ROOT / "scripts" / "output" / f"{args.model}_current_vs_pre_mod"
    ).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    checked = spanning = discrepancies = 0

    for muscle in muscles:
        current_actuator = require_id(
            current_model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle
        )
        pre_mod_actuator = require_id(
            pre_mod_model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle
        )
        current_tendon = current_model.actuator_trnid[current_actuator, 0]
        pre_mod_tendon = pre_mod_model.actuator_trnid[pre_mod_actuator, 0]
        for current_joint, pre_mod_joint in joint_pairs:
            checked += 1
            current_joint_id = require_id(
                current_model, mujoco.mjtObj.mjOBJ_JOINT, current_joint
            )
            pre_mod_joint_id = require_id(
                pre_mod_model, mujoco.mjtObj.mjOBJ_JOINT, pre_mod_joint
            )
            if not (
                has_moment_arm(
                    current_model,
                    current_tendon,
                    current_joint_id,
                    args.eps,
                    args.influence_tol,
                )
                or has_moment_arm(
                    pre_mod_model,
                    pre_mod_tendon,
                    pre_mod_joint_id,
                    args.eps,
                    args.influence_tol,
                )
            ):
                continue
            spanning += 1
            curves = {
                "Current": analyze_pair(
                    current_model, muscle, current_joint, args.samples, args.eps, args.activation
                ),
                "Pre-mod": analyze_pair(
                    pre_mod_model, muscle, pre_mod_joint, args.samples, args.eps, args.activation
                ),
            }
            differences = curve_differences(curves["Current"], curves["Pre-mod"])
            if not curves_disagree(differences, args.percent_tol, args.force_tol):
                continue

            discrepancies += 1
            output = output_dir / safe_filename(f"{muscle}_{pre_mod_joint}.svg")
            save_plot(curves, muscle, pre_mod_joint, args.activation, output, args.model)
            failed_limits = []
            if differences["angle_ranges"]:
                failed_limits.append("joint ranges differ")
            if differences["moment_percent"] > args.percent_tol:
                failed_limits.append(
                    f"moment arm {differences['moment_percent']:.3g}%"
                )
            if differences["length_percent"] > args.percent_tol:
                failed_limits.append(f"MTU length {differences['length_percent']:.3g}%")
            if differences["force_absolute"] > args.force_tol:
                failed_limits.append(
                    f"force {differences['force_absolute']:.3g} N absolute"
                )
            print(
                f"discrepancy: {muscle} @ {pre_mod_joint} "
                f"[{', '.join(failed_limits)}] -> {output.name}"
            )

    print("\nComparison complete")
    print(f"Selected model            : {args.model}")
    print(f"Common muscles             : {len(muscles)}")
    print(f"Candidate muscle/joints   : {checked}")
    print(f"Spanning muscle/joints    : {spanning}")
    print(f"Discrepancies saved       : {discrepancies}")
    print(f"Output directory          : {output_dir}")


if __name__ == "__main__":
    main()
