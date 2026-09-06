"""Plot MyoFullBody elbow moment arms against Pigeon et al. data."""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
from pathlib import Path
import mujoco

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
    "ytick.major.size": 3,
    "xtick.major.width": 0.8,
    "ytick.major.width": 0.8,
    "legend.frameon": False,
    "axes.grid": False,
    "savefig.transparent": True,
    "savefig.dpi": 600
})


REPO_ROOT = Path(__file__).resolve().parents[2]
xml_path = REPO_ROOT / "musclemimic_models" / "model" / "body" / "myofullbody.xml"
sample_points = 200
eps = 1e-5


MUSCLE_SIM_MAP = {
    "BRD":      {"joint": "elbow_flexion", "sim": {"BRD_l": {"tendon": "BRD_tendon_left",  "joint": "elbow_flex_l"},
                                                   "BRD": {"tendon": "BRD_tendon", "joint": "elbow_flex_r"}}},
    "BIClong":  {"joint": "elbow_flexion", "sim": {"BIClong_l": {"tendon": "BIClong_tendon_left",  "joint": "elbow_flex_l"},
                                                   "BIClong": {"tendon": "BIClong_tendon", "joint": "elbow_flex_r"}}},
    "BRA":      {"joint": "elbow_flexion", "sim": {"BRA_l": {"tendon": "BRA_tendon_left",  "joint": "elbow_flex_l"},
                                                   "BRA": {"tendon": "BRA_tendon", "joint": "elbow_flex_r"}}},
    "ECRL":     {"joint": "elbow_flexion", "sim": {"ECRL_l": {"tendon": "ECRL_tendon_left",  "joint": "elbow_flex_l"},
                                                   "ECRL": {"tendon": "ECRL_tendon", "joint": "elbow_flex_r"}}},
    "FCR":      {"joint": "elbow_flexion", "sim": {"FCR_l": {"tendon": "FCR_tendon_left",  "joint": "elbow_flex_l"},
                                                   "FCR": {"tendon": "FCR_tendon", "joint": "elbow_flex_r"}}},
    "ECU":      {"joint": "elbow_flexion", "sim": {"ECU_l": {"tendon": "ECU_tendon_left",  "joint": "elbow_flex_l"},
                                                   "ECU": {"tendon": "ECU_tendon", "joint": "elbow_flex_r"}}},
    "ANC":      {"joint": "elbow_flexion", "sim": {"ANC_l": {"tendon": "ANC_tendon_left",  "joint": "elbow_flex_l"},
                                                   "ANC": {"tendon": "ANC_tendon", "joint": "elbow_flex_r"}}},
    "TRIlong":  {"joint": "elbow_flexion", "sim": {"TRIlong_l": {"tendon": "TRIlong_tendon_left",  "joint": "elbow_flex_l"},
                                                   "TRIlong": {"tendon": "TRIlong_tendon", "joint": "elbow_flex_r"}}},
}

SCRIPT_DIR = Path(__file__).resolve().parent
csv_path = SCRIPT_DIR / "data" / "pigeon_datasets.csv"

raw = pd.read_csv(csv_path, header=None)
muscle_names = raw.iloc[0].tolist()
xy_names = raw.iloc[1].tolist()

header = []
current = None
for m, xy in zip(muscle_names, xy_names):
    if isinstance(m, str) and m.strip() != "":
        current = m.strip()
    xy = str(xy).strip()
    if current and xy in ("X", "Y"):
        header.append(f"{current}_{xy}")
    else:
        header.append(None)

df = raw.iloc[2:].copy()
df.columns = header
df = df.apply(pd.to_numeric, errors="coerce")

muscles = sorted({col.split("_")[0] for col in df.columns if col})

# -----------------------------------------------------------
# 2. EXPERIMENTAL SPLINE FIT
# -----------------------------------------------------------
raw_points = {}
smooth_exp = {}
all_x = []

for m in muscles:
    x = df[f"{m}_X"].dropna().values
    y = df[f"{m}_Y"].dropna().values
    n = min(len(x), len(y))
    x = x[:n]
    y = y[:n]

    raw_points[m] = (x, y)
    all_x.extend(x)

xmin, xmax = min(all_x), max(all_x)
angle_grid = np.linspace(xmin, xmax, 300)

for m in muscles:
    x, y = raw_points[m]
    idx = np.argsort(x)
    xs, ys = x[idx], y[idx]
    spline = UnivariateSpline(xs, ys, s=len(xs) * 3.0)
    smooth_exp[m] = spline(angle_grid)

# -----------------------------------------------------------
# 3. MUJOCO MOMENT ARM COMPUTATION
# -----------------------------------------------------------
model = mujoco.MjModel.from_xml_path(str(xml_path))
data = mujoco.MjData(model)

def mj_moment_arm_curve(tendon, joint):
    tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon)
    joint_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    qpos_id   = model.jnt_qposadr[joint_id]

    jrange = model.jnt_range[joint_id]
    qr = np.linspace(jrange[0], jrange[1], sample_points)

    moment_list = []
    for q in qr:
        data.qpos[:] = 0.0

        data.qpos[qpos_id] = q - eps
        mujoco.mj_forward(model, data)
        L1 = data.ten_length[tendon_id]

        data.qpos[qpos_id] = q + eps
        mujoco.mj_forward(model, data)
        L2 = data.ten_length[tendon_id]

        r = -(L2 - L1) / (2 * eps)
        moment_list.append(r)

    return qr * 180/np.pi, np.array(moment_list) * 1000


# -----------------------------------------------------------
# 4. PER-MUSCLE PLOTS
# -----------------------------------------------------------
OUT = REPO_ROOT / "scripts" / "output" / "experimental_validation" / "elbow"
OUT.mkdir(parents=True, exist_ok=True)

for m in muscles:
    print(f"[INFO] Plotting {m}...")
    cfg = MUSCLE_SIM_MAP[m]

    exp_x, exp_y = raw_points[m]
    exp_smooth = smooth_exp[m]

    plt.figure(figsize=(5,4))

    # raw + smooth experimental
    plt.scatter(exp_x, exp_y, s=12, color='green', label=f"{m} Pigeon(1996)", alpha=0.8)
    plt.plot(angle_grid, exp_smooth, linestyle=":", color='green', alpha=0.9, label=f"{m} (Exp Smooth)")

    # MuJoCo curves
    for sim_name, info in cfg["sim"].items():
        ang, mm = mj_moment_arm_curve(info["tendon"], info["joint"])
        style = "-" if sim_name.endswith("_l") else "--"
        color = "blue" if sim_name.endswith("_l") else "red"
        plt.plot(ang, mm, linestyle=style, color=color, linewidth=2,
                 label=f"{sim_name} (MyoFullBody)")

    plt.xlabel(f"{cfg['joint']} Angle (deg)")
    plt.ylabel("Moment Arm (mm)")
    plt.title(f"{m} — Moment Arm Comparison")
    plt.grid(True)
    plt.legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.02,1))

    plt.tight_layout()
    plt.savefig(OUT / f"moment_arm_{m}.png", dpi=300)
    plt.close()

print("[DONE] All muscles plotted.")
