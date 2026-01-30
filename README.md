# MuscleMimic Models

Musclemimic_models is part of the [**MuscleMimic**](https://github.com/amathislab/musclemimic) research project, in which we created physiologically realistic, muscle-driven musculoskeletal models built on top of [MyoSuite](https://github.com/myohub/myosuite).  This repository is designed to provide users with two musculoskeletal models: BimanualMuscle and MyoFullBody, that could be used together or independently from the Musclemimic pipeline. 

---

## Demos of capable motions

MyoFullBody enables realistic full-body motion control with pure muscle actuation. Below are example motions demonstrating the model's capabilities, all policies were trained with MuscleMimic.
<table>
  <tr>
    <td align="center" width="50%">
      <b>Running</b><br>
      Full-body locomotion with coordinated muscle actuation.<br>
      <video src="https://github.com/user-attachments/assets/323cdf33-95e9-41a5-a8a0-846df99c5958" width="320" controls></video>
    </td>
    <td align="center" width="50%">
      <b>Air Kick</b><br>
      Dynamic kicking motion with balance control.<br>
      <video src="https://github.com/user-attachments/assets/5198b62b-46f0-4d06-b541-6172a412cc4e" width="320" controls></video>
    </td>
  </tr>
</table>

Below we showcase a walking motion and extract the corresponding synthetic muscle activation patterns for three leg muscles along two gait cycles.
<video src="https://github.com/user-attachments/assets/da545e08-b241-451b-b02d-d3ad3e1ee41d" controls></video>
    
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


## Acknowledgements
The models in this repository are based on [MyoSuite](https://github.com/myohub/myosuite). 

