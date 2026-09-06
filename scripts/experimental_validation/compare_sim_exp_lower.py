"""Plot MyoFullBody lower-limb moment arms against published datasets."""

import os
import mujoco
import numpy as np
import matplotlib.pyplot as plt
from xml.etree import ElementTree as ET
import seaborn as sns
from pathlib import Path
import scipy.io
import matplotlib.ticker as mticker

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

REPO_ROOT = Path(__file__).resolve().parents[2]
xml_path = REPO_ROOT / "musclemimic_models" / "model" / "body" / "myofullbody.xml"
SCRIPT_DIR = Path(__file__).resolve().parent
mat_path = (SCRIPT_DIR / "data" / "S3_Datasets.mat").as_posix()
OUTPUT_DIR = REPO_ROOT / "scripts" / "output" / "experimental_validation" / "lower"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

eps = 1e-5
sample_points = 30

MUSCLE_SIM_MAP = {

    # ---------------- BICEPS FEMORIS ----------------
    "Biceps femoris (long)": {
        "joint": "Knee",
        "sim": {
            "bflh_l": {"joint": "knee_angle_l", "tendon": "bflh_l_tendon"},
            "bflh_r": {"joint": "knee_angle_r", "tendon": "bflh_r_tendon"},
        },
    },

    "Biceps femoris (short)": {
        "joint": "Knee",
        "sim": {
            "bfsh_l": {"joint": "knee_angle_l", "tendon": "bfsh_l_tendon"},
            "bfsh_r": {"joint": "knee_angle_r", "tendon": "bfsh_r_tendon"},
        },
    },

    # default BF if unspecified
    "Biceps femoris": {
        "joint": "Knee",
        "sim": {
            "bflh_l": {"joint": "knee_angle_l", "tendon": "bflh_l_tendon"},
            "bflh_r": {"joint": "knee_angle_r", "tendon": "bflh_r_tendon"},
        },
    },

    # ---------------- EXTENSORS ---------------------
    "Extensor digitorum longus": {
        "joint": "Ankle",
        "sim": {
            "edl_l": {"joint": "ankle_angle_l", "tendon": "edl_l_tendon"},
            "edl_r": {"joint": "ankle_angle_r", "tendon": "edl_r_tendon"},
        },
    },

    "Extensor hallucis longus": {
        "joint": "Ankle",
        "sim": {
            "ehl_l": {"joint": "ankle_angle_l", "tendon": "ehl_l_tendon"},
            "ehl_r": {"joint": "ankle_angle_r", "tendon": "ehl_r_tendon"},
        },
    },

    # ---------------- FLEXORS -----------------------
    "Flexor digitorum longus": {
        "joint": "Ankle",
        "sim": {
            "fdl_l": {"joint": "ankle_angle_l", "tendon": "fdl_l_tendon"},
            "fdl_r": {"joint": "ankle_angle_r", "tendon": "fdl_r_tendon"},
        },
    },

    "Flexor hallucis longus": {
        "joint": "Ankle",
        "sim": {
            "fhl_l": {"joint": "ankle_angle_l", "tendon": "fhl_l_tendon"},
            "fhl_r": {"joint": "ankle_angle_r", "tendon": "fhl_r_tendon"},
        },
    },

    # ---------------- GASTROCNEMIUS -----------------
    "Gastrocnemius lateralis": {
        "joint": "Knee",
        "sim": {
            "gaslat_l": {"joint": "knee_angle_l", "tendon": "gaslat_l_tendon"},
            "gaslat_r": {"joint": "knee_angle_r", "tendon": "gaslat_r_tendon"},
        },
    },

    "Gastrocnemius medialis": {
        "joint": "Knee",
        "sim": {
            "gasmed_l": {"joint": "knee_angle_l", "tendon": "gasmed_l_tendon"},
            "gasmed_r": {"joint": "knee_angle_r", "tendon": "gasmed_r_tendon"},
        },
    },

    # ---------------- GLUTEUS MAXIMUS --------------
    "Gluteus maximus": {
        "joint": "Hip",
        "sim": {
            "glmax1_l": {"joint": "hip_flexion_l", "tendon": "glmax1_l_tendon"},
            "glmax1_r": {"joint": "hip_flexion_r", "tendon": "glmax1_r_tendon"},
            "glmax2_l": {"joint": "hip_flexion_l", "tendon": "glmax2_l_tendon"},
            "glmax2_r": {"joint": "hip_flexion_r", "tendon": "glmax2_r_tendon"},
            "glmax3_l": {"joint": "hip_flexion_l", "tendon": "glmax3_l_tendon"},
            "glmax3_r": {"joint": "hip_flexion_r", "tendon": "glmax3_r_tendon"},
        },
    },

    # ---------------- GLUTEUS MEDIUS ---------------
    "Gluteus medius": {
        "joint": "Hip",
        "sim": {
            "glmed1_l": {"joint": "hip_flexion_l", "tendon": "glmed1_l_tendon"},
            "glmed1_r": {"joint": "hip_flexion_r", "tendon": "glmed1_r_tendon"},
            "glmed2_l": {"joint": "hip_flexion_l", "tendon": "glmed2_l_tendon"},
            "glmed2_r": {"joint": "hip_flexion_r", "tendon": "glmed2_r_tendon"},
            "glmed3_l": {"joint": "hip_flexion_l", "tendon": "glmed3_l_tendon"},
            "glmed3_r": {"joint": "hip_flexion_r", "tendon": "glmed3_r_tendon"},
        },
    },

    # ---------------- OTHER LISTED MUSCLES --------
    "Gracilis": {
        "joint": "Knee",
        "sim": {
            "grac_l": {"joint": "knee_angle_l", "tendon": "grac_l_tendon"},
            "grac_r": {"joint": "knee_angle_r", "tendon": "grac_r_tendon"},
        },
    },

    "Piriformis": {
        "joint": "Hip",
        "sim": {
            "piri_l": {"joint": "hip_flexion_l", "tendon": "piri_l_tendon"},
            "piri_r": {"joint": "hip_flexion_r", "tendon": "piri_r_tendon"},
        },
    },

    "Psoas": {
        "joint": "Hip",
        "sim": {
            "psoas_l": {"joint": "hip_flexion_l", "tendon": "psoas_l_tendon"},
            "psoas_r": {"joint": "hip_flexion_r", "tendon": "psoas_r_tendon"},
        },
    },

    "Rectus femoris": {
        "joint": "Knee",
        "sim": {
            "recfem_l": {"joint": "knee_angle_l", "tendon": "recfem_l_tendon"},
            "recfem_r": {"joint": "knee_angle_r", "tendon": "recfem_r_tendon"},
        },
    },

    "Sartorius": {
        "joint": "Knee",
        "sim": {
            "sart_l": {"joint": "knee_angle_l", "tendon": "sart_l_tendon"},
            "sart_r": {"joint": "knee_angle_r", "tendon": "sart_r_tendon"},
        },
    },

    "Semimembranosus": {
        "joint": "Knee",
        "sim": {
            "semimem_l": {"joint": "knee_angle_l", "tendon": "semimem_l_tendon"},
            "semimem_r": {"joint": "knee_angle_r", "tendon": "semimem_r_tendon"},
        },
    },

    "Semitendinosus": {
        "joint": "Knee",
        "sim": {
            "semiten_l": {"joint": "knee_angle_l", "tendon": "semiten_l_tendon"},
            "semiten_r": {"joint": "knee_angle_r", "tendon": "semiten_r_tendon"},
        },
    },

    "Soleus": {
        "joint": "Ankle",
        "sim": {
            "soleus_l": {"joint": "ankle_angle_l", "tendon": "soleus_l_tendon"},
            "soleus_r": {"joint": "ankle_angle_r", "tendon": "soleus_r_tendon"},
        },
    },

    "Tensor fasciae latae": {
        "joint": "Hip",
        "sim": {
            "tfl_l": {"joint": "hip_flexion_l", "tendon": "tfl_l_tendon"},
            "tfl_r": {"joint": "hip_flexion_r", "tendon": "tfl_r_tendon"},
        },
    },

    "Tibialis anterior": {
        "joint": "Ankle",
        "sim": {
            "tibant_l": {"joint": "ankle_angle_l", "tendon": "tibant_l_tendon"},
            "tibant_r": {"joint": "ankle_angle_r", "tendon": "tibant_r_tendon"},
        },
    },

    "Tibialis posterior": {
        "joint": "Ankle",
        "sim": {
            "tibpost_l": {"joint": "ankle_angle_l", "tendon": "tibpost_l_tendon"},
            "tibpost_r": {"joint": "ankle_angle_r", "tendon": "tibpost_r_tendon"},
        },
    },

    "Vastus intermedius": {
        "joint": "Knee",
        "sim": {
            "vasint_l": {"joint": "knee_angle_l", "tendon": "vasint_l_tendon"},
            "vasint_r": {"joint": "knee_angle_r", "tendon": "vasint_r_tendon"},
        },
    },

    "Vastus lateralis": {
        "joint": "Knee",
        "sim": {
            "vaslat_l": {"joint": "knee_angle_l", "tendon": "vaslat_l_tendon"},
            "vaslat_r": {"joint": "knee_angle_r", "tendon": "vaslat_r_tendon"},
        },
    },

    "Vastus medialis": {
        "joint": "Knee",
        "sim": {
            "vasmed_l": {"joint": "knee_angle_l", "tendon": "vasmed_l_tendon"},
            "vasmed_r": {"joint": "knee_angle_r", "tendon": "vasmed_r_tendon"},
        },
    },
}

# ----------------------------------------------------------------------
# Fix slashes in includes
# ----------------------------------------------------------------------
def fix_slashes_in_includes(xml_path_in, output_path):
    tree = ET.parse(xml_path_in)
    root = tree.getroot()
    for elem in root.iter("include"):
        if "file" in elem.attrib:
            elem.attrib["file"] = elem.attrib["file"].replace("\\", "/")
    tree.write(output_path)

# Repository paths already use POSIX separators; never rewrite the source model.

# ----------------------------------------------------------------------
# Equality parsing (from expanded XML)
# ----------------------------------------------------------------------
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

# ----------------------------------------------------------------------
# MAT helpers
# ----------------------------------------------------------------------
def load_moment_arm_dataset(path):
    mat = scipy.io.loadmat(path, squeeze_me=True, struct_as_record=False)
    return mat["dataset_momentArm"]

def get_joint_name(primary_dof_string):
    return primary_dof_string.split()[0]

def extract_numeric_data(measureValue):
    extracted = []

    if isinstance(measureValue, np.ndarray) and measureValue.ndim == 2:
        containers = [measureValue]
    elif isinstance(measureValue, np.ndarray) and measureValue.ndim == 1:
        containers = list(measureValue)
    elif isinstance(measureValue, (list, tuple)):
        containers = list(measureValue)
    else:
        containers = [measureValue]

    for block in containers:
        if block is None or np.isscalar(block):
            continue

        if isinstance(block, (list, tuple)):
            subblocks = block
        else:
            subblocks = [block]

        for sub in subblocks:
            arr = np.asarray(sub)
            if arr.ndim != 2:
                continue
            if arr.shape[0] == 3:
                extracted.append((arr[0, :], arr[2, :]))
            elif arr.shape[1] == 3:
                extracted.append((arr[:, 0], arr[:, 2]))
    return extracted

def parse_entry(entry):
    joint = get_joint_name(entry.primaryDoF)
    numeric = extract_numeric_data(entry.measureValue)
    return {
        "muscle": entry.measureObject,
        "joint": joint,
        "reference": entry.reference,
        "blocks": numeric
    }

def get_mat_curves(dataset, muscle, joint):
    curves = []
    for entry in dataset:
        if entry.measureObject != muscle:
            continue
        if get_joint_name(entry.primaryDoF) != joint:
            continue

        parsed = parse_entry(entry)
        for angle, moment in parsed["blocks"]:
            curves.append({
                "angle_deg": np.asarray(angle),
                "moment_mm": np.asarray(moment),
                "reference": parsed["reference"],
            })
    return curves

# ----------------------------------------------------------------------
# Load model + expanded XML + equality maps
# ----------------------------------------------------------------------
model = mujoco.MjModel.from_xml_path(str(xml_path))
data = mujoco.MjData(model)

expanded_xml = SCRIPT_DIR / ".expanded_model_lower_tmp.xml"
mujoco.mj_saveLastXML(str(expanded_xml), model)

slave_to_master, master_to_slaves = parse_joint_equalities(expanded_xml, model)
expanded_xml.unlink(missing_ok=True)

# ----------------------------------------------------------------------
# Plotting helper
# ----------------------------------------------------------------------
def plot_one_muscle(muscle_name, cfg):
    plt.figure(figsize=(4.5, 3))

    # ------------------------------------------------------------------
    # A. SIMULATION CURVES (left/right tendons, hardcoded in cfg["sim"])
    # ------------------------------------------------------------------
    for sim_name, info in cfg["sim"].items():
        tendon_name = info["tendon"]
        joint_name  = info["joint"]

        tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
        joint_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)

        # Decide which joint we actually drive:
        # If this joint is a slave → drive its master.
        if joint_id in slave_to_master:
            master_joint_id = slave_to_master[joint_id][0]
        else:
            master_joint_id = joint_id

        master_qpos_id = model.jnt_qposadr[master_joint_id]

        jrange_raw = model.jnt_range[joint_id]
        joint_range = np.linspace(jrange_raw[0], jrange_raw[1], sample_points)

        moment_list = []
        for q in joint_range:
            data.qpos[:] = 0.0

            # q - eps
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

        moments_mm = np.array(moment_list) * 1000
        angles_deg = joint_range * 180 / np.pi

        # left = blue solid, right = red dashed
        if sim_name.endswith("_l"):
            color = "blue"
            style = "-"
        else:
            color = "red"
            style = "--"

        plt.plot(
            angles_deg, moments_mm,
            color=color, linestyle=style, linewidth=2,
            label=f"{sim_name} (myofullbody)"
        )

    # ------------------------------------------------------------------
    # B. EXPERIMENTAL CURVES (MAT dataset)
    # ------------------------------------------------------------------
    ds = load_moment_arm_dataset(mat_path)
    mat_curves = get_mat_curves(ds, muscle_name, cfg["joint"])

    cmap = plt.cm.tab10
    for i, curve in enumerate(mat_curves):
        angle = np.asarray(curve["angle_deg"])
        moment = np.asarray(curve["moment_mm"])

        if np.allclose(angle, angle[0]):
            continue

        c = cmap(i % 10)
        plt.plot(
            curve["angle_deg"],
            curve["moment_mm"],
            linestyle=":", color=c, alpha=0.9,
            label=f"{curve['reference']} (exp)"
        )

    # Remove duplicate legend entries
    handles, labels = plt.gca().get_legend_handles_labels()
    #unique = dict(zip(labels, handles))

    plt.legend(handles, labels, fontsize=7, loc="upper left", bbox_to_anchor=(1.02, 1))

    plt.title(f"{muscle_name} — Moment Arm")
    plt.xlabel(f"{cfg['joint']} Angle (deg)")
    plt.ylabel("Moment Arm (mm)")
    plt.grid(True)

    plt.savefig(OUTPUT_DIR / f"moment_arm_{muscle_name}.png", bbox_inches="tight")
    plt.close()


for muscle_name, cfg in MUSCLE_SIM_MAP.items():
    print(f"Plotting {muscle_name}...")
    plot_one_muscle(muscle_name, cfg)

print("Done! All lower-limb plots saved to:", OUTPUT_DIR)
