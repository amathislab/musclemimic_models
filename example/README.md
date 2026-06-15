# Examples

Runnable Jupyter notebooks for `musclemimic_models`.

| Notebook | What it covers |
|----------|----------------|
| [`01_load_and_visualize.ipynb`](01_load_and_visualize.ipynb) | Load the `bimanual` and `myofullbody` models, inspect their dimensions (joints, degrees of freedom, muscles, and tendons), render static images and multiple camera angles, run a muscle-driven rollout with animation, and open the interactive viewer. |
| [`02_single_arm_bimanual.ipynb`](02_single_arm_bimanual.ipynb) | Create a single-arm variant of the bimanual model with **`remove_arm`** (remove the unused arm) or **`freeze_arm`** (keep the unused arm visible but welded in place), both via `MjSpec`. It also includes an action-masking variant that avoids recompilation. |

## Setup

The package itself only needs `mujoco`. The notebooks additionally use `matplotlib`,
`scipy`, and `numpy`, and run inside Jupyter — all bundled in the `dev` extra:

```bash
# from the repo root
pip install -e ".[dev]"     # installs the package + notebook deps (matplotlib, scipy, numpy, jupyter)

jupyter notebook example/
```

## Rendering on a headless server

Each notebook's first cell sets the off-screen GL backend:

```python
import os
os.environ.setdefault("MUJOCO_GL", "egl")   # headless servers
```

- On a **server with no display**, keep this (`egl`, or `osmesa` for pure-CPU rendering).
- On a **laptop/desktop with a screen**, you can remove the line to use the default
  (`glfw`) backend; only then will `mujoco.viewer.launch(...)` open an interactive window.

> **Tip: image size.** MuJoCo's default off-screen framebuffer is 640x480. To render larger
> (e.g. a tall full-body portrait), enlarge it before creating the `Renderer`:
> `model.vis.global_.offheight = 640`. The notebooks wrap this in a `make_renderer` helper.
