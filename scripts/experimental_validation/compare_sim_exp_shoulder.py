"""Plot MyoFullBody shoulder moment arms against Ackland et al. data."""

import os
import mujoco
import numpy as np
import matplotlib.pyplot as plt
from xml.etree import ElementTree as ET
import seaborn as sns
from pathlib import Path
import pandas as pd

sns.set_theme(style="white", context="talk")

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Arial"],
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 9,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "legend.fontsize": 8,
    "axes.linewidth": 0.8,
    "lines.linewidth": 1.5,
    "figure.dpi": 300,
    "xtick.direction": "out",
    "ytick.direction": "out",
    "xtick.major.size": 3,
    "ytick.major.width": 0.8,
    "ytick.major.size": 3,
    "legend.frameon": False,
    "axes.grid": False,
    "savefig.transparent": True,
    "savefig.dpi": 600
})

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIR = REPO_ROOT / "scripts" / "output" / "experimental_validation" / "shoulder"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

xml_path = REPO_ROOT / "musclemimic_models" / "model" / "body" / "myofullbody.xml"
ackland_csv = SCRIPT_DIR / "data" / "shoulder_dataset_Ackland.csv"


eps = 1e-5
sample_points = 40

# ================================================================
# Load Ackland dataset (tidy format, abduction only)
# Columns: muscle, region, joint_action, angle_deg, moment_arm_mm
# ================================================================
ack = pd.read_csv(ackland_csv)
ack = ack[ack["joint_action"] == "abduction"].copy()

# ================================================================
# Shoulder group → regions (Ackland) → sim muscles
# ================================================================
# region names are from your tidy pipeline: lowercased, underscores
SHOULDER_GROUPS = {
    "teres_major": {
        "regions": ["teres_major"],
        "sim": ["TMAJ"],
    },
    "supraspinatus": {
        "regions": ["anterior_supraspinatus", "posterior_supraspinatus"],
        "sim": ["SUPSP"],
    },
    "subscapularis": {
        "regions": [
            "superior_subscapularis",
            "middle_subscapularis",
            "inferior_subscapularis",
        ],
        "sim": ["SUBSC"],
    },
    "infraspinatus": {
        "regions": ["superior_infraspinatus", "inferior_infraspinatus"],
        "sim": ["INFSP"],
    },
    "teres_minor": {
        "regions": ["teres_minor"],
        "sim": ["TMIN"],
    },
    "anterior_deltoid": {
        "regions": ["anterior_deltoid"],
        "sim": ["DELT1"],
    },
    "middle_deltoid": {
        "regions": ["middle_deltoid"],
        "sim": ["DELT2"],
    },
    "posterior_deltoid": {
        "regions": ["posterior_deltoid"],
        "sim": ["DELT3"],
    },
    "superior_lat_dorsi": {
        "regions": ["superior_lat_dorsi"],
        "sim": ["LAT1"],
    },
    "middle_lat_dorsi": {
        "regions": ["middle_lat_dorsi"],
        "sim": ["LAT2"],
    },
    "inferior_lat_dorsi": {
        "regions": ["inferior_lat_dorsi"],
        "sim": ["LAT3"],
    },
    "superior_pec_major": {
        "regions": ["superior_pec_major"],
        "sim": ["PECM1"],
    },
    "middle_pec_major": {
        "regions": ["middle_pec_major"],
        "sim": ["PECM2"],
    },
    "inferior_pec_major": {
        "regions": ["inferior_pec_major"],
        "sim": ["PECM3"],
    },
}

# ================================================================
# Fix slashes in includes (same helper as lower-limb)
# ================================================================
def fix_slashes_in_includes(xml_path_in, output_path):
    tree = ET.parse(xml_path_in)
    root = tree.getroot()
    for elem in root.iter("include"):
        if "file" in elem.attrib:
            elem.attrib["file"] = elem.attrib["file"].replace("\\", "/")
    tree.write(output_path)

# Repository paths already use POSIX separators; never rewrite the source model.

# ================================================================
# Equality parsing (from expanded XML)
# ================================================================
def parse_joint_equalities(expanded_xml_path, model):
    """
    Returns:
      slave_to_master: {slave_joint_id: (master_joint_id, coeffs)}
      master_to_slaves: {master_joint_id: [(slave_joint_id, coeffs), ...]}
    """
    tree = ET.parse(expanded_xml_path)
    root = tree.getroot()

    slave_to_master = {}
    master_to_slaves = {}

    for elem in root.iter():
        # In MuJoCo XML, equality joints are <equality><joint .../></equality>
        if not elem.tag.endswith("joint"):
            continue

        slave_name = elem.get("joint1")
        master_name = elem.get("joint2")
        if slave_name is None or master_name is None:
            continue

        try:
            slave_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, slave_name)
            master_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, master_name)
        except Exception:
            continue

        if slave_id < 0 or master_id < 0:
            continue

        poly = elem.get("polycoef")
        if poly is None:
            coeffs = [0.0, 1.0]  # default: slave = master
        else:
            coeffs = [float(x) for x in poly.split()]

        slave_to_master[slave_id] = (master_id, coeffs)
        master_to_slaves.setdefault(master_id, []).append((slave_id, coeffs))

    return slave_to_master, master_to_slaves


def apply_local_eq_constraints(master_id, data, model, master_to_slaves):
    """
    Update only the slaves of this master joint:
        slave = Σ coeff[p] * master^p
    """
    if master_id not in master_to_slaves:
        return
    master_qpos_id = model.jnt_qposadr[master_id]
    mval = data.qpos[master_qpos_id]
    for slave_id, coeffs in master_to_slaves[master_id]:
        slave_qpos_id = model.jnt_qposadr[slave_id]
        sval = 0.0
        for p, c in enumerate(coeffs):
            sval += c * (mval ** p)
        data.qpos[slave_qpos_id] = sval

# ================================================================
# Load model + build equality maps
# ================================================================
model = mujoco.MjModel.from_xml_path(str(xml_path))
data = mujoco.MjData(model)

expanded_xml = SCRIPT_DIR / "expanded_model_shoulder_tmp.xml"
mujoco.mj_saveLastXML(str(expanded_xml), model)

slave_to_master, master_to_slaves = parse_joint_equalities(str(expanded_xml), model)
expanded_xml.unlink(missing_ok=True)

def compute_sim_curve(sim_muscle, side):
    """
    side: 'r' or 'l'
    Right:
        tendon: <muscle>_tendon
        joint : shoulder_elv_r
    Left:
        tendon: <muscle>_tendon_left
        joint : shoulder_elv_l
    """

    if side == "r":
        tendon_name = f"{sim_muscle}_tendon"
        joint_name  = "shoulder_elv_r"
    else:
        tendon_name = f"{sim_muscle}_tendon_left"
        joint_name  = "shoulder_elv_l"

    # Look up IDs
    tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
    joint_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,  joint_name)

    if tendon_id < 0 or joint_id < 0:
        print(f"[WARNING] sim muscle={sim_muscle}, side={side} tendon='{tendon_name}', joint='{joint_name}' not found")
        return None

    # Handle joint equality constraints (drive master)
    if joint_id in slave_to_master:
        master_joint_id = slave_to_master[joint_id][0]
    else:
        master_joint_id = joint_id

    master_qpos_id = model.jnt_qposadr[master_joint_id]

    # Use range of the *side-specific joint*
    jrange_raw = model.jnt_range[joint_id]
    joint_range = np.linspace(jrange_raw[0], jrange_raw[1], sample_points)

    moment_list = []
    for q in joint_range:

        # q - eps
        data.qpos[:] = 0.0
        data.qpos[master_qpos_id] = q - eps
        apply_local_eq_constraints(master_joint_id, data, model, master_to_slaves)
        mujoco.mj_forward(model, data)
        L1 = data.ten_length[tendon_id]

        # q + eps
        data.qpos[:] = 0.0
        data.qpos[master_qpos_id] = q + eps
        apply_local_eq_constraints(master_joint_id, data, model, master_to_slaves)
        mujoco.mj_forward(model, data)
        L2 = data.ten_length[tendon_id]

        r = -(L2 - L1) / (2 * eps)
        moment_list.append(r)

    moments_mm = np.array(moment_list) * 1000.0
    angles_deg = joint_range * 180.0 / np.pi
    return angles_deg, moments_mm

POLY_ORDER = 5

for group_name, cfg in SHOULDER_GROUPS.items():
    regions = cfg["regions"]
    sim_muscles = cfg["sim"]

    ack_sub = ack[ack["region"].isin(regions)].copy()
    if ack_sub.empty:
        print(f"[Skipping] No Ackland data for {group_name}")
        continue

    plt.figure(figsize=(5, 4))
    cmap = [
        "#009E73",  # green
        "#D55E00",  # orange
        "#CC79A7",  # pink
        "#E69F00",  # goldenrod
        "#0072B2",  # blue
        "#56B4E9",  # light blue
        "#F0E442",  # yellow
    ]

    # ------------------------------
    # A. Experimental (Ackland)
    # ------------------------------
    for i, region_name in enumerate(sorted(ack_sub["region"].unique())):
        df_r = ack_sub[ack_sub["region"] == region_name]
        x = df_r["angle_deg"].values
        y = df_r["moment_arm_mm"].values

        color = cmap[i % 10]

        # Scatter
        plt.scatter(x[::2], y[::2], color=color, s=5, label=f"{region_name} (Ackland 2008)")

        # Fit
        if len(x) >= POLY_ORDER + 1:
            coeffs = np.polyfit(x, y, POLY_ORDER)
            poly = np.poly1d(coeffs)
            xx = np.linspace(min(x), max(x), 300)
            yy = poly(xx)
            plt.plot(xx, yy, color=color, linestyle=":",
                     linewidth=1.5, label=f"{region_name} fit")

    # ------------------------------
    # B. Simulation: RIGHT + LEFT
    # ------------------------------
    for sim_muscle in sim_muscles:

        # Left side (dashed)
        sim_l = compute_sim_curve(sim_muscle, "l")
        if sim_l is not None:
            q, rvals = sim_l
            plt.plot(q, rvals, linewidth=2,  color="blue",
                     label=f"{sim_muscle}_l (myofullbody)")
        
        # Right side (solid)
        sim_r = compute_sim_curve(sim_muscle, "r")
        if sim_r is not None:
            q, rvals = sim_r
            plt.plot(q, rvals, linewidth=2, linestyle="--", color="red",
                     label=f"{sim_muscle}_r (myofullbody)")

    # ------------------------------
    # C. Formatting
    # ------------------------------
    title_pretty = group_name.replace("_", " ").title()
    plt.title(f"{title_pretty} — Abduction Moment Arm")
    plt.xlabel("Shoulder Elevation Angle (deg)")
    plt.ylabel("Moment Arm (mm)")

    # Unique legend entries
    handles, labels = plt.gca().get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    plt.legend(uniq.values(), uniq.keys(),
               fontsize=7, loc="upper left", bbox_to_anchor=(1.02, 1))

    plt.tight_layout()

    fname = f"{group_name}_abduction_validation.png"
    plt.savefig(OUTPUT_DIR / fname, bbox_inches="tight")
    plt.close()

    print(f"[Saved] {fname}")

print("Done! All shoulder L+R plots saved to:", OUTPUT_DIR)
