# MuscleMimic Models


<p align="center">
  <img
    src="https://github.com/user-attachments/assets/25ca8915-6c44-40f2-8bac-2e4b2b706de9"
    width="60%"
  />
</p>

Musclemimic_models is part of the [**MuscleMimic**](https://github.com/amathislab/musclemimic) research project, in which we created physiologically realistic, muscle-driven musculoskeletal models built on top of [MyoSuite](https://github.com/myohub/myosuite).  This repository is designed to provide users with two musculoskeletal models: BimanualMuscle and MyoFullBody, that could be used together or independently from the Musclemimic pipeline. 

MyoFullBody enables realistic full-body motion control with pure muscle actuation. Below are example fullbody motions demonstrating the model's capabilities, all policies were trained with MuscleMimic.

<table>
  <tr>
    <td align="center" width="50%">
      <b>Backwards Walking</b>
      <video src="https://github.com/user-attachments/assets/c413767a-e8d2-4a73-80d3-bc58bd8f90b7" width="320" controls></video>
    </td>
    <td align="center" width="50%">
      <b>Walking Running</b>
      <video src="https://github.com/user-attachments/assets/305e34e2-49ff-4f1b-a62d-2bc8376dcf4f" width="320" controls></video>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <b>Walking Turning</b>
      <video src="https://github.com/user-attachments/assets/1fb2ea5d-1435-45d8-8cce-094500cfe76a" width="320" controls></video>
    </td>
    <td align="center" width="50%">
      <b>Dancing</b>
      <video src="https://github.com/user-attachments/assets/12abc09d-4d36-49d3-9138-65e4ee4fa4bd" width="320" controls></video>
    </td>
  </tr>
</table>

<br/>
<br/>
<p align="center">
  <strong><em>
    MyoFullbody also allows accurate kinematics when trained with MuscleMimic on AMASS data
  </em></strong>
</p>

![Untitled design (1)](https://github.com/user-attachments/assets/13312e4b-6b52-4c8c-8707-499510db676d)

<br/>
<br/>

**BimanualMuscle** focuses on upper-limb musculoskeletal control, enabling faster training convergence while preserving full finger articulation capabilities.
(The videos shown below were recorded with finger actuation disabled)



<br/>
<br/>

<table>
  <tr>
    <td align="center" width="50%">
      <b>Lifting Box</b>
      <video src="https://github.com/user-attachments/assets/f31f1c0a-0652-47d2-b9b4-2d6abd2cab84" width="320" controls></video>
    </td>
    <td align="center" width="50%">
      <b>Waving</b>
      <video src="https://github.com/user-attachments/assets/dcdb0e93-28c9-46a5-b4be-3fc22cebf6f7" width="320" controls></video>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <b>Drinking Water</b>
      <video src="https://github.com/user-attachments/assets/b2b97ed0-cc9e-40bc-a18c-538250081674" width="320" controls></video>
    </td>
    <td align="center" width="50%">
      <b>Jumpingjack</b>
      <video src="https://github.com/user-attachments/assets/b9716c13-da84-4571-bc0f-ae47018d6b8d" width="320" controls></video>
    </td>
  </tr>
</table>

---

## Musculoskeletal Models

Both musculoskeletal models are built on MyoSuite components, combining **[MyoArm](https://github.com/MyoHub/myo_sim/tree/main/arm)**, **[MyoLegs](https://github.com/MyoHub/myo_sim/tree/main/leg)**, and **[MyoTorso](https://github.com/MyoHub/myo_sim/tree/main/torso)** models with Hill-type muscle actuators in MuJoCo. This enables studying motor control at the **neuromuscular level** and realistic muscle output, rather than via idealized joint torque controllers.

### Environment Summary

| Model           | Type        | Joints | DoF | Muscles | Focus                        |
|-----------------|-------------|--------|---------|---------|------------------------------|
| BimanualMuscle  | Fixed-base  | 76 (36*)    | 126 (64*)     | 54 (14*) | Upper-body manipulation      |
| MyoFullBody     | Free-root   | 123 (83*)    | 416 (354*)    | 72 (32*) | Locomotion and manipulation    |

##### $^*$ denotes configurations with finger muscles temporarily disabled. 

### BimanualMuscle Environment

The **BimanualMuscle** environment is designed for **upper-body manipulation task**. Explicit contacts are enabled in between both arms and with the thorax.
<p align="center">
  <img width="2820" height="800" alt="BimanualMuscle" src="https://github.com/user-attachments/assets/67e68c50-43dd-4f0f-845e-53cd3a984f1f" />
</p>

### MyoFullBody Environment

The **MyoFullBody** environment provides a **comprehensive full-body musculoskeletal system** with full biomechanical detail and rich contact dynamics, suitable for **locomotion**, **manipulation**, and **whole-body imitation**. We explicitly enable additional collision pairs, such as leg–leg, arm–leg, foot-foot, to capture the required self-contact behavior, including bimanual interactions. 


<p align="center">
  <img width="2820" height="1515" alt="MyoFullBody" src="https://github.com/user-attachments/assets/067986b7-00a7-4461-9b53-dc7a72fd21ed" />
</p>

---

## Getting Started

### Prerequisites

The minimum required MuJoCo version for both models is `mujoco==3.2.1`. To use spec with the main Musclemimic environment, please use `mujoco>=3.3.0`. 

### Overview

The structure of the Musclemimic model is as follows. We use MyoFullBody as an example. 
```
musclemimic_models/
└── model/
    ├── arm/
    │   ├── assets/
    │   └── myoarm_bimanual.xml
    ├── body/
    │   └── myofullbody.xml
    ├── head/
    │   └── assets/
    ├── leg/
    │   └── assets/
    ├── torso/
    │   └── assets/
    ├── meshes/
    └── scene/
└── tests/
```
- `assets/` : includes both the kinematics chain files and the assets definition files for each body segment that its under. 
- `meshes/` : shared mesh files used across models for bones and skulls
- `scene/` : MJCF “scene” files used in both MSK as backgrounds
- `arm/`, `body/`, `head/`, `leg/`, `torso/` : model components and their associated assets/
- `*.xml` : MJCF model definition(s) (e.g., `myofullbody.xml`, `myoarm_bimanual.xml`)
- `test/`: testing files for symmetry between bodies, geoms, sites and muscle

### Usage

#### Via Pypi
Install:

```bash
pip install musclemimic-models
```


#### Via `git clone`
Clone and install editable (recommended for development):

```bash
git clone https://github.com/amathislab/musclemimic_models.git
cd musclemimic_models
pip install -e .
```

---
### MSK Model Refinement and Validation
**Muscle Jump and Symmetry**

While building MyoFullBody and BimanualMuscle, we corrected left–right limb asymmetries and addressed several unexpected muscle-jumping behaviors. A few representative fixes are shown below.

<p align="center">
  <img src="https://github.com/user-attachments/assets/c4772223-e655-46c4-9105-2db4938e6450" width="45%">
  <img src="https://github.com/user-attachments/assets/97c6645c-651a-4796-8b80-7d112ef34f91" width="45%">
</p>


**Muscle Validation**

We also cross-validate the current model using previously published cadaver studies and MRI data. A few illustrative examples are included here.

<p align="center">
  <img src="https://github.com/user-attachments/assets/02e187ea-9257-4429-afeb-59bd5b6ff5fd" width="35%">
  <img src="https://github.com/user-attachments/assets/c3d2fe42-ebfa-49b7-b53e-302ed87b53e5" width="35%">
  <img src="https://github.com/user-attachments/assets/e2d43cda-a49f-466e-8381-7f86d608111d" width="25%">
</p>


## License
Musclemimic models are licensed under the [Apache License](https://github.com/amathislab/myofullbody/blob/main/LICENSE).


## Citation

```bibtex
@article{li2026musclemimic,
  title={Towards Embodied AI with MuscleMimic: Unlocking full-body musculoskeletal motor learning at scale},
  author={Li, Chengkun and Wang, Cheryl and Ziliotto, Bianca and Simos, Merkourios and Durandau, Guillaume and Mathis, Alexander},
  year={2026}
}
```

## Acknowledgements
The models in this repository are based on [MyoSuite](https://github.com/myohub/myosuite). 





