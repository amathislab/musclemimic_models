import unittest

import mujoco
import numpy as np

import musclemimic_models as mm


class JointConstraintRangeTest(unittest.TestCase):
    def test_left_knee_translation2_stays_within_range(self):
        model, _ = mm.load("myofullbody")

        equality_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_EQUALITY,
            "knee_angle_translation2_constraint_l",
        )
        self.assertNotEqual(equality_id, -1)

        target_joint_id = int(model.eq_obj1id[equality_id])
        driver_joint_id = int(model.eq_obj2id[equality_id])

        self.assertEqual(
            mujoco.mj_id2name(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                target_joint_id,
            ),
            "knee_angle_translation2_l",
        )
        self.assertEqual(
            mujoco.mj_id2name(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                driver_joint_id,
            ),
            "knee_angle_l",
        )

        driver_min, driver_max = model.jnt_range[driver_joint_id]
        target_min, target_max = model.jnt_range[target_joint_id]

        driver_qpos_address = model.jnt_qposadr[driver_joint_id]
        target_qpos_address = model.jnt_qposadr[target_joint_id]

        driver_reference = model.qpos0[driver_qpos_address]
        target_reference = model.qpos0[target_qpos_address]

        driver_positions = np.linspace(
            driver_min,
            driver_max,
            num=10_001,
        )
        driver_offsets = driver_positions - driver_reference

        coefficients = model.eq_data[equality_id, :5]
        target_positions = target_reference + sum(
            coefficient * driver_offsets**degree
            for degree, coefficient in enumerate(coefficients)
        )

        tolerance = 1e-9

        self.assertGreaterEqual(
            float(target_positions.min()),
            float(target_min - tolerance),
        )
        self.assertLessEqual(
            float(target_positions.max()),
            float(target_max + tolerance),
        )
        
    def test_knee_translation2_displacements_are_mirrored(self):
        model, _ = mm.load("myofullbody")

        target_positions = {}
        target_axes = {}

        for side in ("r", "l"):
            equality_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_EQUALITY,
                f"knee_angle_translation2_constraint_{side}",
            )
            self.assertNotEqual(equality_id, -1)

            target_joint_id = int(model.eq_obj1id[equality_id])
            driver_joint_id = int(model.eq_obj2id[equality_id])

            driver_min, driver_max = model.jnt_range[driver_joint_id]

            driver_reference = model.qpos0[
                model.jnt_qposadr[driver_joint_id]
            ]
            target_reference = model.qpos0[
                model.jnt_qposadr[target_joint_id]
            ]

            driver_values = np.linspace(
                driver_min,
                driver_max,
                num=10_001,
            )
            driver_offsets = driver_values - driver_reference

            coefficients = model.eq_data[equality_id, :5]
            target_positions[side] = target_reference + sum(
                coefficient * driver_offsets**degree
                for degree, coefficient in enumerate(coefficients)
            )
            target_axes[side] = model.jnt_axis[target_joint_id]

        reflection = np.array([1.0, 1.0, -1.0])

        np.testing.assert_allclose(
            target_axes["l"],
            target_axes["r"] * reflection,
            atol=1e-7,
            rtol=0.0,
        )

        right_displacements = (
            target_positions["r"][:, None] * target_axes["r"]
        )
        expected_left_displacements = right_displacements * reflection
        actual_left_displacements = (
            target_positions["l"][:, None] * target_axes["l"]
        )

        np.testing.assert_allclose(
            actual_left_displacements,
            expected_left_displacements,
            atol=1e-9,
            rtol=0.0,
        )


if __name__ == "__main__":
    unittest.main()
