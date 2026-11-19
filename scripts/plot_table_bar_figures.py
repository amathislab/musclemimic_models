"""Create bar-plot versions of the paper's diagnostic and training tables."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch


OUTPUT_DIR = Path(__file__).resolve().parent / "output" / "table_bar_figures"

COLORS = {
    "MyoFullBody": "#0072B2",
    "MuscleMimic": "#0072B2",
    "MS-Human-700": "#D55E00",
    "MyoSim-FB": "#009E73",
}


def set_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "Arial",
            "font.size": 15,
            "axes.labelsize": 16,
            "axes.titlesize": 16,
            "xtick.labelsize": 14,
            "ytick.labelsize": 14,
            "legend.fontsize": 15,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.axisbelow": True,
            "axes.grid": True,
            "axes.grid.axis": "y",
            "grid.color": "#DDE3E8",
            "grid.linewidth": 0.8,
            "svg.fonttype": "none",
            "pdf.fonttype": 42,
        }
    )


def annotate_bars(
    ax: plt.Axes,
    bars,
    labels: list[str] | None = None,
    fontsize: float = 13,
) -> None:
    labels = labels or [f"{bar.get_height():g}" for bar in bars]
    heights = [bar.get_height() for bar in bars if np.isfinite(bar.get_height())]
    offset = (max(heights) if heights else 1) * 0.025
    for bar, label in zip(bars, labels):
        if label == "NA":
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                offset,
                label,
                ha="center",
                va="bottom",
                fontsize=fontsize,
                fontweight="bold",
            )
        else:
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + offset,
                label,
                ha="center",
                va="bottom",
                fontsize=fontsize,
            )


def annotate_grouped_bars(
    ax: plt.Axes, bars, labels: list[str], model_index: int
) -> None:
    """Vertically stagger adjacent labels so similar values do not collide."""
    heights = [bar.get_height() for bar in bars]
    offset = max(heights) * (0.065 if model_index == 0 else 0.02)
    for bar, label in zip(bars, labels):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + offset,
            label,
            ha="center",
            va="bottom",
            fontsize=11.5,
        )


def finish(fig: plt.Figure, stem: str) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for suffix in ("png", "pdf", "svg"):
        fig.savefig(
            OUTPUT_DIR / f"{stem}.{suffix}",
            dpi=300,
            bbox_inches="tight",
            facecolor="white",
        )
    plt.close(fig)


def plot_diagnostics() -> None:
    models = ["MyoSim-FB", "MyoFullBody", "MS-Human-700"]
    metrics = [
        ("Max. normalized FL jump", [1.31, 0.0, 1.40], ["1.31", "0", "1.40"]),
        ("Max. normalized MA jump", [0.91, 0.0, 1.00], ["0.91", "0", "1.00"]),
        ("FL jumps (%)", [4.62, 0.0, 2.95], ["4.62%", "0.00%", "2.95%"]),
        ("MA jumps (%)", [4.62, 0.0, 3.32], ["4.62%", "0.00%", "3.32%"]),
        ("Mean asymmetry NRMSE", [0.30, 0.0, 0.51], ["0.30", "0", "0.51"]),
        ("Bilateral asymmetry (%)", [15.97, 0.0, 1.48], ["15.97%", "0.00%", "1.48%"]),
    ]
    fig, axes = plt.subplots(1, 6, figsize=(24, 4.8))
    x = np.arange(len(models))
    for index, (ax, (title, values, labels)) in enumerate(zip(axes.flat, metrics)):
        plotted = np.nan_to_num(values, nan=0.0)
        bars = ax.bar(
            x,
            plotted,
            width=0.68,
            color=[COLORS[model] for model in models],
            edgecolor="white",
            linewidth=1.2,
        )
        if np.isnan(values[1]):
            bars[1].set_facecolor("white")
            bars[1].set_edgecolor(COLORS["MyoFullBody"])
            bars[1].set_hatch("///")
        annotate_bars(ax, bars, labels)
        ax.set_title(f"{title} ↓", fontweight="bold", pad=12)
        ax.set_xticks(x, ["MyoSim-FB", "MyoFullBody", "MS-Human-700"], rotation=16)
        top = np.nanmax(values)
        ax.set_ylim(0, top * 1.20 if top else 1)
    fig.legend(
        handles=[Patch(facecolor=COLORS[m], label=m) for m in models],
        loc="upper center",
        ncol=3,
        frameon=False,
        bbox_to_anchor=(0.5, 1.015),
    )
    fig.subplots_adjust(top=0.76, bottom=0.22, wspace=0.38)
    finish(fig, "muscle_path_diagnostics_bars")


def plot_retargeting() -> None:
    models = ["MyoSim-FB", "MuscleMimic"]
    metrics = [
        ("Max. tendon jump", [0.265, 0.040], ["0.265", "0.040"]),
        ("Jump rate (%)", [86.61, 19.08], ["86.61%", "19.08%"]),
        ("Site RMSE (m)", [0.036, 0.038], ["0.036", "0.038"]),
        ("Failures (#)", [177, 6], ["177", "6"]),
    ]
    fig, axes = plt.subplots(4, 1, figsize=(12, 24))
    x = np.arange(2)
    for index, (ax, (title, values, labels)) in enumerate(zip(axes, metrics)):
        bars = ax.bar(
            x,
            values,
            width=0.64,
            color=[COLORS[model] for model in models],
            edgecolor="white",
            linewidth=2.2,
        )
        annotate_bars(ax, bars, labels, fontsize=40)
        ax.set_title(f"{title} ↓", fontsize=52, fontweight="bold", pad=28)
        ax.set_xticks(x, models)
        ax.tick_params(axis="both", labelsize=44, width=2.5, length=10)
        ax.set_ylim(0, max(values) * 1.20)
    fig.subplots_adjust(top=0.97, bottom=0.045, left=0.16, right=0.98, hspace=0.76)
    finish(fig, "retargeting_summary_bars")


def plot_training() -> None:
    models = ["MyoSim-FB", "MuscleMimic"]
    tasks = ["Bimanual (2B)", "Full body (4B)"]
    # Shape: model x task x evaluation seed. The MuscleMimic rows correspond
    # to MyoBimanualArm and MyoFullBody in the supplied validation table.
    metrics = [
        (
            "Success rate (%)",
            100.0
            * (
                1.0
                - np.asarray(
                    [
                        [[0.0, 0.0, 0.0], [0.009259, 0.018519, 0.0]],
                        [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                    ]
                )
            ),
            "↑",
            1,
        ),
        (
            "Frame coverage (%)",
            np.asarray(
                [
                    [[1.0, 1.0, 1.0], [0.998951, 0.995212, 1.0]],
                    [[1.0, 1.0, 1.0], [1.0, 1.0, 1.0]],
                ]
            )
            * 100.0,
            "↑",
            1,
        ),
        (
            "Kinematic error (°)",
            np.rad2deg(
                np.asarray(
                    [
                        [
                            [0.164958, 0.167553, 0.166347],
                            [0.179427, 0.179201, 0.178335],
                        ],
                        [
                            [0.094880, 0.094896, 0.094714],
                            [0.175352, 0.175061, 0.174788],
                        ],
                    ]
                )
            ),
            "↓",
            2,
        ),
        (
            "Site error (cm)",
            np.asarray(
                [
                    [
                        [0.013367, 0.013383, 0.013376],
                        [0.074265, 0.070741, 0.074402],
                    ],
                    [
                        [0.012744, 0.012774, 0.012746],
                        [0.057969, 0.057775, 0.057337],
                    ],
                ]
            )
            * 100.0,
            "↓",
            2,
        ),
        (
            "Steps (M)",
            np.asarray([[510.2, 1006.0], [186.1, 739.08]]),
            "↓",
            2,
        ),
        (
            "Time (h)",
            np.asarray([[4.83, 10.09], [1.29, 7.46]]),
            "↓",
            2,
        ),
    ]
    fig, axes = plt.subplots(1, 6, figsize=(24, 5.2))
    x = np.arange(len(tasks))
    width = 0.34
    for ax, (title, seed_values, direction, precision) in zip(axes, metrics):
        has_seed_points = seed_values.ndim == 3
        if has_seed_points:
            means = seed_values.mean(axis=2)
            errors = seed_values.std(axis=2, ddof=1)
        else:
            means = seed_values
            errors = None
        for model_index, model in enumerate(models):
            positions = x + (model_index - 0.5) * width
            bars = ax.bar(
                positions,
                means[model_index],
                width=width,
                color=COLORS[model],
                edgecolor="white",
                linewidth=1.0,
                yerr=None if errors is None else errors[model_index],
                capsize=3,
                error_kw={"elinewidth": 1.2, "capthick": 1.2},
            )
            labels = [f"{value:.{precision}f}" for value in means[model_index]]
            annotate_grouped_bars(ax, bars, labels, model_index)
            if has_seed_points:
                for task_index, position in enumerate(positions):
                    jitter = np.asarray([-0.045, 0.0, 0.045])
                    ax.scatter(
                        position + jitter,
                        seed_values[model_index, task_index],
                        s=28,
                        facecolor="white",
                        edgecolor="#222222",
                        linewidth=0.9,
                        zorder=5,
                    )
        ax.set_title(f"{title} {direction}", fontweight="bold", pad=12)
        ax.set_xticks(x, tasks, rotation=18)
        ax.set_ylim(0, means.max() * 1.23)
    fig.legend(
        handles=[Patch(facecolor=COLORS[m], label=m) for m in models],
        loc="upper center",
        ncol=2,
        frameon=False,
        bbox_to_anchor=(0.5, 1.03),
    )
    fig.subplots_adjust(top=0.76, bottom=0.22, wspace=0.36)
    finish(fig, "training_summary_bars")


def main() -> None:
    set_style()
    plot_diagnostics()
    plot_retargeting()
    plot_training()
    print(OUTPUT_DIR)


if __name__ == "__main__":
    main()
