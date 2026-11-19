"""Summarize curve jumps and bilateral asymmetry for the three muscle models.

The analysis uses the conservative one-to-one muscle and joint mappings from
``compare_myofullbody_mshuman700.py``.  A force--length (FD) jump is the
Euclidean distance between adjacent samples after independently scaling length
and force to [0, 1].  A moment-arm (MA) jump is the adjacent difference after
scaling the moment arm to [0, 1].  Both use the streaming EMA detector from
``musclemimic_dev``'s ``msk_metrics.tendon_jump_max``.

Before each MuJoCo forward pass, dependent joint positions are evaluated from
their joint-equality polynomials, matching myo_sim's symmetry-test sampling.

Bilateral NRMSE combines the left/right errors in normalized moment arm, MTU
length, and force.  Each channel is normalized by its pooled bilateral range.
The reported mean bilateral NRMSE is calculated only over pairs whose NRMSE
exceeds ``--asymmetry-tol``.

Run from anywhere with:

    uv run --extra dev python -u scripts/summarize_model_quality.py
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

import compare_myofullbody_mshuman700 as comparison


@dataclass(frozen=True)
class ModelInfo:
    label: str
    model: mujoco.MjModel


def detected_jump_max(
    steps: np.ndarray,
    jump_factor: float,
    minimum: float,
    ema_alpha: float,
) -> tuple[float, bool]:
    """Apply the msk_metrics streaming EMA rule to normalized curve steps."""
    if not len(steps):
        return 0.0, False
    ema = float(steps[0])
    flagged_max = 0.0
    for index, step in enumerate(steps):
        step = float(step)
        if index:
            ema = (1.0 - ema_alpha) * ema + ema_alpha * step
        threshold = max(jump_factor * ema, minimum)
        if step > threshold:
            flagged_max = max(flagged_max, step)
    return flagged_max, flagged_max > 0.0


def detected_jump_indices(
    steps: np.ndarray,
    jump_factor: float,
    minimum: float,
    ema_alpha: float,
) -> np.ndarray:
    """Return step indices flagged by the msk_metrics streaming EMA rule."""
    if not len(steps):
        return np.empty(0, dtype=int)
    ema = float(steps[0])
    flagged = []
    for index, step in enumerate(steps):
        step = float(step)
        if index:
            ema = (1.0 - ema_alpha) * ema + ema_alpha * step
        if step > max(jump_factor * ema, minimum):
            flagged.append(index)
    return np.asarray(flagged, dtype=int)


def normalized(values: np.ndarray) -> np.ndarray:
    """Min-max normalize a curve, returning zero for a constant curve."""
    values = np.asarray(values, dtype=float)
    span = float(np.ptp(values))
    if span <= np.finfo(float).eps:
        return np.zeros_like(values)
    return (values - np.min(values)) / span


def curve_jump_metrics(
    curve: dict[str, np.ndarray],
    jump_factor: float,
    minimum: float,
    ema_alpha: float,
    refined_curve: dict[str, np.ndarray] | None = None,
    convergence_ratio: float = 0.75,
) -> dict[str, float | bool]:
    ma_steps = np.abs(np.diff(normalized(curve["moment_arms"])))
    length_steps = np.diff(normalized(curve["lengths"]))
    force_steps = np.diff(normalized(curve["forces"]))
    fd_steps = np.hypot(length_steps, force_steps)
    max_fd_jump, fd_jump = detected_jump_max(
        fd_steps, jump_factor, minimum, ema_alpha
    )
    max_ma_jump, ma_jump = detected_jump_max(
        ma_steps, jump_factor, minimum, ema_alpha
    )
    result: dict[str, float | bool] = {
        "max_normalized_fd_jump": max_fd_jump,
        "max_normalized_ma_jump": max_ma_jump,
        "fd_jump": fd_jump,
        "ma_jump": ma_jump,
        "fd_refinement_ratio": float("nan"),
        "ma_refinement_ratio": float("nan"),
        "fd_steep_continuous": False,
        "ma_steep_continuous": False,
    }
    if refined_curve is None:
        return result

    refined_ma_steps = np.abs(np.diff(normalized(refined_curve["moment_arms"])))
    refined_fd_steps = np.hypot(
        np.diff(normalized(refined_curve["lengths"])),
        np.diff(normalized(refined_curve["forces"])),
    )
    for prefix, flagged, coarse_steps, refined_steps in (
        ("fd", fd_jump, fd_steps, refined_fd_steps),
        ("ma", ma_jump, ma_steps, refined_ma_steps),
    ):
        if not flagged:
            continue
        coarse_max = float(np.max(coarse_steps, initial=0.0))
        refined_max = float(np.max(refined_steps, initial=0.0))
        ratio = refined_max / coarse_max if coarse_max else 0.0
        result[f"{prefix}_refinement_ratio"] = ratio
        if ratio < convergence_ratio:
            result[f"{prefix}_jump"] = False
            result[f"max_normalized_{prefix}_jump"] = 0.0
            result[f"{prefix}_steep_continuous"] = True
    return result


def save_flagged_plot(
    curve: dict[str, np.ndarray],
    muscle: str,
    joint: str,
    output: Path,
    jump_factor: float,
    minimum: float,
    ema_alpha: float,
) -> None:
    """Plot a flagged raw FD and MA curve with detected FD intervals marked."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    length_steps = np.diff(normalized(curve["lengths"]))
    force_steps = np.diff(normalized(curve["forces"]))
    fd_steps = np.hypot(length_steps, force_steps)
    jumps = detected_jump_indices(fd_steps, jump_factor, minimum, ema_alpha)
    points = np.unique(np.concatenate((jumps, jumps + 1)))

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    axes[0].plot(curve["lengths"], curve["forces"], color="#2A9D55", linewidth=2)
    axes[0].scatter(
        curve["lengths"][points], curve["forces"][points],
        color="crimson", edgecolor="white", linewidth=0.6, s=42, zorder=3,
        label="Flagged interval",
    )
    axes[0].set(xlabel="MTU length (m)", ylabel="Force (N)", title="Force–length curve")
    axes[0].legend(frameon=False)

    axes[1].plot(curve["angles"], curve["moment_arms"], color="#386CB0", linewidth=2)
    axes[1].scatter(
        curve["angles"][points], curve["moment_arms"][points],
        color="crimson", edgecolor="white", linewidth=0.6, s=42, zorder=3,
    )
    axes[1].set(
        xlabel=f"{joint} angle (rad)", ylabel="Moment arm (m)",
        title="Moment arm",
    )
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
    fig.suptitle(
        f"MyoFullBody: {muscle} at {joint}\n"
        f"maximum flagged normalized FD jump = {np.max(fd_steps[jumps]):.6g}"
    )
    fig.tight_layout()
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)


def save_flagged_ma_plot(
    curve: dict[str, np.ndarray],
    muscle: str,
    joint: str,
    output: Path,
    jump_factor: float,
    minimum: float,
    ema_alpha: float,
) -> None:
    """Plot a flagged MA curve and corresponding raw force--length curve."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ma_steps = np.abs(np.diff(normalized(curve["moment_arms"])))
    jumps = detected_jump_indices(ma_steps, jump_factor, minimum, ema_alpha)
    points = np.unique(np.concatenate((jumps, jumps + 1)))

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    axes[0].plot(curve["angles"], curve["moment_arms"], color="#386CB0", linewidth=2)
    axes[0].scatter(
        curve["angles"][points], curve["moment_arms"][points],
        color="crimson", edgecolor="white", linewidth=0.6, s=42, zorder=3,
        label="Flagged interval",
    )
    axes[0].set(
        xlabel=f"{joint} angle (rad)", ylabel="Moment arm (m)",
        title="Moment arm",
    )
    axes[0].legend(frameon=False)

    axes[1].plot(curve["lengths"], curve["forces"], color="#2A9D55", linewidth=2)
    axes[1].scatter(
        curve["lengths"][points], curve["forces"][points],
        color="crimson", edgecolor="white", linewidth=0.6, s=42, zorder=3,
    )
    axes[1].set(xlabel="MTU length (m)", ylabel="Force (N)", title="Force–length curve")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
    fig.suptitle(
        f"MyoFullBody: {muscle} at {joint}\n"
        f"maximum flagged normalized MA jump = {np.max(ma_steps[jumps]):.6g}"
    )
    fig.tight_layout()
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)


def bilateral_nrmse(
    right: dict[str, np.ndarray], left: dict[str, np.ndarray]
) -> float:
    """Return one equally weighted NRMSE across MA, length, and force."""
    channel_mses = []
    for channel in ("moment_arms", "lengths", "forces"):
        right_values = np.asarray(right[channel], dtype=float)
        left_values = np.asarray(left[channel], dtype=float)
        pooled = np.concatenate((right_values, left_values))
        scale = float(np.ptp(pooled))
        if scale <= np.finfo(float).eps:
            channel_mses.append(0.0)
        else:
            channel_mses.append(float(np.mean(((right_values - left_values) / scale) ** 2)))
    return math.sqrt(float(np.mean(channel_mses)))


def write_csv(path: Path, fields: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def fmt(value: float | None, percent: bool = False) -> str:
    if value is None:
        return "NA"
    return f"{value:.2f}%" if percent else f"{value:.6g}"


def write_markdown(
    path: Path,
    rows: list[dict[str, object]],
    samples: int,
    jump_factor: float,
    minimum: float,
    ema_alpha: float,
    convergence_ratio: float,
    asymmetry_tol: float,
) -> None:
    lines = [
        "# Three-model curve-quality summary",
        "",
        "| Model | Maximum normalized FD jump | Maximum normalized MA jump | FD jumps | MA jumps | Mean bilateral NRMSE among asymmetric pairs | Bilateral asymmetry |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {model} | {fd_max} | {ma_max} | {fd_pct} ({fd_count}/{curve_count}) "
            "| {ma_pct} ({ma_count}/{curve_count}) | {nrmse} ({asym_count} pairs) "
            "| {asym_pct} ({asym_count}/{bilateral_count}) |".format(
                model=row["model"],
                fd_max=fmt(float(row["max_normalized_fd_jump"])),
                ma_max=fmt(float(row["max_normalized_ma_jump"])),
                fd_pct=fmt(float(row["fd_jump_percent"]), percent=True),
                ma_pct=fmt(float(row["ma_jump_percent"]), percent=True),
                nrmse=fmt(row["mean_bilateral_nrmse_asymmetric"]),
                asym_pct=fmt(float(row["bilateral_asymmetry_percent"]), percent=True),
                fd_count=row["fd_count"],
                ma_count=row["ma_count"],
                curve_count=row["curve_count"],
                asym_count=row["asym_count"],
                bilateral_count=row["bilateral_count"],
            )
        )
    lines.extend([
        "",
        "## Definitions",
        "",
        f"Curves were sampled at {samples} evenly spaced points over each joint range, giving a normalized joint-position interval of 1/{samples - 1} ({100 / (samples - 1):.4g}% of the range). FD is the two-dimensional force–length curve; length and force are each min-max normalized before adjacent Euclidean distances are calculated. MA is independently min-max normalized before adjacent differences are calculated.",
        "",
        f"The jump detector follows msk_metrics.tendon_jump_max: its EMA starts at the first step and is updated with α = {ema_alpha:g}; a step is flagged when it exceeds max({jump_factor:g} × EMA, {minimum:g}). Maximum jump columns contain the largest flagged event, or zero when none is detected. Percentages use the model's mapped, spanning muscle–joint curves as the denominator.",
        "",
        f"Each preliminary event is resampled at twice the resolution. If the refined maximum step is less than {convergence_ratio:g} times the original, it is classified as a steep continuous curve rather than a discontinuity. Smooth changes approach a ratio of 0.5; true jumps approach 1.0.",
        "",
        "Before every MuJoCo forward pass, active joint equality constraints are evaluated explicitly: each dependent joint is set from its master joint using the compiled equality polynomial.",
        "",
        f"Bilateral NRMSE is the root mean square of the normalized left/right errors across moment arm, MTU length, and force, with each channel normalized by its pooled bilateral range. A bilateral pair is asymmetric when NRMSE > {asymmetry_tol:g}. The table averages NRMSE only across those asymmetric pairs.",
        "",
    ])
    path.write_text("\n".join(lines))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=201)
    parser.add_argument("--eps", type=float, default=1e-5)
    parser.add_argument("--activation", type=float, default=1.0)
    parser.add_argument("--influence-tol", type=float, default=1e-6)
    parser.add_argument("--jump-factor", type=float, default=10.0)
    parser.add_argument(
        "--min-normalized-jump", type=float, default=0.01,
        help="Minimum normalized curve step (default: 0.01, or 1%% of its range)",
    )
    parser.add_argument("--ema-alpha", type=float, default=0.01)
    parser.add_argument("--convergence-ratio", type=float, default=0.75)
    parser.add_argument(
        "--asymmetry-tol", type=float, default=0.1,
        help="Bilateral NRMSE above which a pair is asymmetric (default: 0.1)",
    )
    parser.add_argument(
        "--plot-flagged-myofullbody",
        action="store_true",
        help="Save diagnostic plots for MyoFullBody curves with flagged FD jumps",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=comparison.REPO_ROOT / "scripts/output/model_quality_summary",
    )
    args = parser.parse_args()
    if args.samples < 3:
        parser.error("--samples must be at least 3")
    if args.eps <= 0 or args.influence_tol < 0:
        parser.error("--eps must be positive and --influence-tol cannot be negative")
    if not 0 <= args.activation <= 1:
        parser.error("--activation must be between 0 and 1")
    if args.jump_factor < 0 or args.min_normalized_jump < 0 or args.asymmetry_tol < 0:
        parser.error("thresholds cannot be negative")
    if not 0 < args.ema_alpha <= 1:
        parser.error("--ema-alpha must be in (0, 1]")
    if not 0 < args.convergence_ratio <= 1:
        parser.error("--convergence-ratio must be in (0, 1]")

    print("Loading MyoFullBody, myo_sim, and MS-Human-700...", flush=True)
    myo = mujoco.MjModel.from_xml_path(str(comparison.MYO_XML))
    pre = mujoco.MjModel.from_xml_path(str(comparison.PRE_MOD_XML))
    ms = mujoco.MjModel.from_xml_path(str(comparison.MS_XML))
    model_infos = (
        ModelInfo("MyoFullBody", myo),
        ModelInfo("myo_sim", pre),
        ModelInfo("MS-Human-700", ms),
    )

    myo_muscles = comparison.names(myo, mujoco.mjtObj.mjOBJ_ACTUATOR, myo.nu)
    ms_muscles = {
        name for name in comparison.names(ms, mujoco.mjtObj.mjOBJ_ACTUATOR, ms.nu)
        if not comparison.is_neck_muscle(name)
    }
    myo_to_ms, _ = comparison.build_mapping(myo_muscles, ms_muscles)
    pre_muscles = set(comparison.names(pre, mujoco.mjtObj.mjOBJ_ACTUATOR, pre.nu))
    muscles = []
    for myo_name, ms_name in myo_to_ms.items():
        pre_name = comparison.pre_mod_muscle_name(myo_name, pre_muscles)
        if pre_name is None:
            continue
        names = {"MyoFullBody": myo_name, "myo_sim": pre_name, "MS-Human-700": ms_name}
        ids = {}
        tendons = {}
        for info in model_infos:
            actuator = comparison.object_id(
                info.model, mujoco.mjtObj.mjOBJ_ACTUATOR, names[info.label]
            )
            ids[info.label] = actuator
            tendons[info.label] = int(info.model.actuator_trnid[actuator, 0])
        if all(tendon >= 0 for tendon in tendons.values()):
            muscles.append((ms_name, names, ids, tendons))

    myo_joints = comparison.names(myo, mujoco.mjtObj.mjOBJ_JOINT, myo.njnt)
    ms_joints = set(comparison.names(ms, mujoco.mjtObj.mjOBJ_JOINT, ms.njnt))
    pre_joints = set(comparison.names(pre, mujoco.mjtObj.mjOBJ_JOINT, pre.njnt))
    myo_dependent = comparison.equality_dependent_joints(myo)
    ms_dependent = comparison.equality_dependent_joints(ms)
    joints = []
    for myo_name in myo_joints:
        ms_name = comparison.JOINT_ALIASES.get(myo_name, myo_name)
        if (
            ms_name not in ms_joints
            or myo_name in myo_dependent
            or ms_name in ms_dependent
            or not comparison.model_joint_is_limited(myo, myo_name)
            or not comparison.model_joint_is_limited(ms, ms_name)
        ):
            continue
        pre_name = comparison.pre_mod_joint_name(myo_name, ms_name, pre_joints)
        if pre_name is None or not comparison.model_joint_is_limited(pre, pre_name):
            continue
        joints.append((ms_name, {
            "MyoFullBody": myo_name, "myo_sim": pre_name, "MS-Human-700": ms_name,
        }))

    curve_rows: list[dict[str, object]] = []
    myofullbody_fd_plots: list[
        tuple[str, str, dict[str, np.ndarray], dict[str, float | bool]]
    ] = []
    bilateral_curves: dict[
        str, dict[tuple[str, str], dict[str, dict[str, np.ndarray]]]
    ] = {info.label: {} for info in model_infos}
    for index, (ms_joint, joint_names) in enumerate(joints, start=1):
        print(f"Sampling joint {index}/{len(joints)}: {ms_joint}", flush=True)
        sampled = {
            info.label: comparison.curves_for_joint(
                info.model, joint_names[info.label], args.samples, args.eps, args.activation
            )
            for info in model_infos
        }
        refined_sampled: dict[
            str, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]
        ] = {}
        for ms_muscle, muscle_names, actuator_ids, tendon_ids in muscles:
            muscle_base, muscle_side = comparison.side_and_base(ms_muscle)
            joint_base, joint_side = comparison.side_and_base(ms_joint)
            if muscle_side and joint_side and muscle_side != joint_side:
                continue
            side = muscle_side or joint_side
            if side not in ("right", "left"):
                continue
            for info in model_infos:
                curve = comparison.select_curve(
                    sampled[info.label], actuator_ids[info.label], tendon_ids[info.label]
                )
                if np.max(np.abs(curve["moment_arms"])) <= args.influence_tol:
                    continue
                preliminary = curve_jump_metrics(
                    curve,
                    args.jump_factor,
                    args.min_normalized_jump,
                    args.ema_alpha,
                )
                if preliminary["fd_jump"] or preliminary["ma_jump"]:
                    if info.label not in refined_sampled:
                        refined_sampled[info.label] = comparison.curves_for_joint(
                            info.model,
                            joint_names[info.label],
                            2 * args.samples - 1,
                            args.eps,
                            args.activation,
                        )
                    refined_curve = comparison.select_curve(
                        refined_sampled[info.label],
                        actuator_ids[info.label],
                        tendon_ids[info.label],
                    )
                    metrics = curve_jump_metrics(
                        curve,
                        args.jump_factor,
                        args.min_normalized_jump,
                        args.ema_alpha,
                        refined_curve,
                        args.convergence_ratio,
                    )
                else:
                    metrics = preliminary
                curve_rows.append({
                    "model": info.label,
                    "muscle": muscle_names[info.label],
                    "joint": joint_names[info.label],
                    "canonical_muscle": ms_muscle,
                    "canonical_joint": ms_joint,
                    **metrics,
                })
                if (
                    args.plot_flagged_myofullbody
                    and info.label == "MyoFullBody"
                    and metrics["fd_jump"]
                ):
                    myofullbody_fd_plots.append((
                        muscle_names[info.label], joint_names[info.label], curve, metrics
                    ))
                bilateral_curves[info.label].setdefault(
                    (muscle_base, joint_base), {}
                )[side] = curve

    bilateral_rows: list[dict[str, object]] = []
    for info in model_infos:
        for (muscle, joint), sides in sorted(bilateral_curves[info.label].items()):
            if "right" not in sides or "left" not in sides:
                continue
            nrmse = bilateral_nrmse(sides["right"], sides["left"])
            bilateral_rows.append({
                "model": info.label,
                "muscle": muscle,
                "joint": joint,
                "bilateral_nrmse": nrmse,
                "asymmetric": nrmse > args.asymmetry_tol,
            })

    summary_rows: list[dict[str, object]] = []
    for info in model_infos:
        model_curves = [row for row in curve_rows if row["model"] == info.label]
        model_bilateral = [row for row in bilateral_rows if row["model"] == info.label]
        asymmetric = [row for row in model_bilateral if row["asymmetric"]]
        curve_count = len(model_curves)
        bilateral_count = len(model_bilateral)
        fd_count = sum(bool(row["fd_jump"]) for row in model_curves)
        ma_count = sum(bool(row["ma_jump"]) for row in model_curves)
        asym_count = len(asymmetric)
        summary_rows.append({
            "model": info.label,
            "max_normalized_fd_jump": max(
                (float(row["max_normalized_fd_jump"]) for row in model_curves), default=0.0
            ),
            "max_normalized_ma_jump": max(
                (float(row["max_normalized_ma_jump"]) for row in model_curves), default=0.0
            ),
            "fd_jump_percent": 100.0 * fd_count / curve_count if curve_count else 0.0,
            "ma_jump_percent": 100.0 * ma_count / curve_count if curve_count else 0.0,
            "mean_bilateral_nrmse_asymmetric": (
                float(np.mean([row["bilateral_nrmse"] for row in asymmetric]))
                if asymmetric else None
            ),
            "bilateral_asymmetry_percent": (
                100.0 * asym_count / bilateral_count if bilateral_count else 0.0
            ),
            "fd_count": fd_count,
            "ma_count": ma_count,
            "curve_count": curve_count,
            "asym_count": asym_count,
            "bilateral_count": bilateral_count,
        })

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if args.plot_flagged_myofullbody:
        plot_dir = output_dir / "myofullbody_flagged_fd"
        plot_dir.mkdir(parents=True, exist_ok=True)
        for old_plot in plot_dir.glob("*.png"):
            old_plot.unlink()
        for muscle, joint, curve, _metrics in myofullbody_fd_plots:
            save_flagged_plot(
                curve,
                muscle,
                joint,
                plot_dir / comparison.safe_name(f"{muscle}__{joint}__fd_jump.png"),
                args.jump_factor,
                args.min_normalized_jump,
                args.ema_alpha,
            )
    summary_fields = list(summary_rows[0]) if summary_rows else []
    write_csv(output_dir / "model_quality_summary.csv", summary_fields, summary_rows)
    write_csv(output_dir / "curve_jump_metrics.csv", [
        "model", "muscle", "joint", "canonical_muscle", "canonical_joint",
        "max_normalized_fd_jump",
        "max_normalized_ma_jump", "fd_jump", "ma_jump",
        "fd_refinement_ratio", "ma_refinement_ratio",
        "fd_steep_continuous", "ma_steep_continuous",
    ], curve_rows)
    write_csv(output_dir / "bilateral_metrics.csv", [
        "model", "muscle", "joint", "bilateral_nrmse", "asymmetric",
    ], bilateral_rows)
    markdown = output_dir / "model_quality_summary.md"
    write_markdown(
        markdown, summary_rows, args.samples, args.jump_factor,
        args.min_normalized_jump, args.ema_alpha, args.convergence_ratio,
        args.asymmetry_tol,
    )
    print(f"\nSummary table: {markdown}")
    print(f"Summary CSV  : {output_dir / 'model_quality_summary.csv'}")
    print(f"Detail CSVs  : {output_dir / 'curve_jump_metrics.csv'}")
    print(f"               {output_dir / 'bilateral_metrics.csv'}")
    if args.plot_flagged_myofullbody:
        print(f"FD plots     : {plot_dir} ({len(myofullbody_fd_plots)} files)")


if __name__ == "__main__":
    main()
