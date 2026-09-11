"""Utilities for analyzing muscle symmetry via moment arm and force-length curves."""

import mujoco
import numpy as np
import matplotlib.pyplot as plt


def parse_model_joint_equalities(model):
    """Return active joint equalities as slave -> (master, polynomial)."""
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


def apply_eq_constraints(data, model, eq_map):
    """Evaluate joint equality polynomials into ``data.qpos``."""
    for slave_id, (master_id, coefficients) in eq_map.items():
        slave_qpos = int(model.jnt_qposadr[slave_id])
        master_qpos = int(model.jnt_qposadr[master_id])
        master_value = float(data.qpos[master_qpos])
        data.qpos[slave_qpos] = sum(
            coefficient * master_value**power
            for power, coefficient in enumerate(coefficients)
        )


def compute_moment_arm_curve(
    model, data, tendon_id, jnt_id, eps=1e-5, n=100, eq_map=None
):
    """Compute moment arm curve for a tendon across a joint's range using finite differences."""
    qpos_id = model.jnt_qposadr[jnt_id]
    q0, q1 = model.jnt_range[jnt_id]
    if q0 == q1:
        return None, None

    qs = np.linspace(q0, q1, n)
    ma = np.zeros_like(qs)
    eq_map = eq_map or {}

    for i, q in enumerate(qs):
        data.qpos[:] = model.qpos0
        data.qpos[qpos_id] = q - eps
        apply_eq_constraints(data, model, eq_map)
        mujoco.mj_forward(model, data)
        L1 = data.ten_length[tendon_id]

        data.qpos[:] = model.qpos0
        data.qpos[qpos_id] = q + eps
        apply_eq_constraints(data, model, eq_map)
        mujoco.mj_forward(model, data)
        L2 = data.ten_length[tendon_id]

        ma[i] = -(L2 - L1) / (2 * eps)

    return qs, ma


def compute_force_length_curve(
    model, data, act_id, jnt_id, activation=1.0, n=100, eq_map=None
):
    """Compute MTU force-length curve for an actuator across a joint's range."""
    qpos_id = model.jnt_qposadr[jnt_id]
    q0, q1 = model.jnt_range[jnt_id]
    if q0 == q1:
        return None, None

    qs = np.linspace(q0, q1, n)
    lengths, forces = [], []
    eq_map = eq_map or {}

    for q in qs:
        data.qpos[:] = model.qpos0
        data.qpos[qpos_id] = q
        apply_eq_constraints(data, model, eq_map)
        data.act[:] = 0.0
        data.act[act_id] = activation
        mujoco.mj_forward(model, data)

        lengths.append(data.actuator_length[act_id])
        forces.append(-data.actuator_force[act_id])

    return np.asarray(lengths), np.asarray(forces)


def plot_pair(
    curves,
    title,
    out_path=None,
    moment_arm_tol=2e-5,
    moment_arm_rtol=5e-4,
    force_rtol=1e-2,
    force_atol=0.1,
):
    """Plot left/right muscle comparison. Saves plot only if discrepancy found."""
    r, l = curves["right"], curves["left"]

    moment_arm_ok = np.allclose(
        r["moment_arms"],
        l["moment_arms"],
        atol=moment_arm_tol,
        rtol=moment_arm_rtol,
    )
    force_difference = np.abs(r["forces"] - l["forces"])
    force_scale = np.maximum(np.abs(r["forces"]), np.abs(l["forces"]))
    force_ok = np.all(
        (force_difference < force_atol)
        | (force_difference < force_rtol * force_scale)
    )
    discrep = not (moment_arm_ok and force_ok)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    for side, style in [("right", "-"), ("left", "dotted")]:
        c = curves[side]
        axes[0].plot(
            c["jnt_range"],
            c["moment_arms"],
            linestyle=style,
            label=c["muscle"],
        )
        axes[1].plot(
            c["mtu_lengths"],
            c["forces"],
            linestyle=style,
            label=c["muscle"],
        )

    # ---- Titles ----
    axes[0].set_title("Moment arm")
    axes[1].set_title("Force–length")

    # ---- Axis labels ----
    axes[0].set_xlabel("Joint angle (rad)")
    axes[0].set_ylabel("Moment arm (m)")

    axes[1].set_xlabel("MTU length (m)")
    axes[1].set_ylabel("Muscle force (N)")

    for ax in axes:
        ax.legend(fontsize=8)
        ax.grid(True)

    fig.suptitle(("✓ OK" if not discrep else "✗ DISCREPANCY") + " – " + title)
    plt.tight_layout()

    if discrep and out_path is not None:
        plt.savefig(out_path, dpi=200, bbox_inches="tight")

    plt.close(fig)
    return not discrep
