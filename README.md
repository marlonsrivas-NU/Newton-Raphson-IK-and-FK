# Franka Panda FK/IK — Product of Exponentials & Damped Least Squares

> Implementation of Forward and Inverse Kinematics for the Franka Emika Panda robot using the Product of Exponentials method and Damped Least Squares solver in MuJoCo.

---

## Overview

This project implements and validates **Forward Kinematics (FK)** and **Inverse Kinematics (IK)** for the 7-DOF Franka Emika Panda robot arm using the **Modern Robotics** framework inside **MuJoCo**.

| Notebook | Description |
|---|---|
| `Forward_Kinematics_Analytically.ipynb` | PoE FK implementation validated against MuJoCo over 100 random configurations |
| `Newton_Raphson_IK.ipynb` | Damped Least Squares IK solver for 7-segment trajectory tracking |

---

## Theory

### Forward Kinematics — Product of Exponentials

The end-effector pose is computed using the space-frame PoE formula:

$$T = e^{[S_1]\theta_1} \cdots e^{[S_n]\theta_n} M$$

where $M \in SE(3)$ is the home configuration pose and $S_i = (\omega_i, v_i)$ is the screw axis for each joint expressed in the world frame.

### Inverse Kinematics — Damped Least Squares

The IK update follows a Newton-Raphson scheme using the **Damped Least Squares (DLS)** method for improved numerical stability near singularities:

$$\Delta q = J_b^T\left(J_b J_b^T + \lambda^2 I\right)^{-1} \mathcal{V}_{err}$$

where $\lambda = 0.05$ is the damping factor and $\mathcal{V}_{err}$ is the body-frame twist error computed from the matrix logarithm:

$$\mathcal{V}_{err} = \left(\log\left(T_{current}^{-1} T_{target}\right)\right)^\vee$$

---

## Requirements

- Python 3.10+
- [MuJoCo](https://github.com/google-deepmind/mujoco)
- [Modern Robotics](https://github.com/NxRLab/ModernRobotics)
- NumPy
- Matplotlib
- Mediapy

Install dependencies:

```bash
pip install mujoco modern-robotics numpy matplotlib mediapy
```

---

## Setup

1. Clone the repository:

```bash
git clone https://github.com/your-username/your-repo-name.git
cd your-repo-name
```

2. Create and activate a virtual environment:

```bash
python -m venv .venv

# Windows
.venv\Scripts\activate

# Mac/Linux
source .venv/bin/activate
```

3. Install dependencies:

```bash
pip install mujoco modern-robotics numpy matplotlib mediapy
```

4. Update the MuJoCo XML path inside the notebooks to match your local path:

```python
model = mujoco.MjModel.from_xml_path(r"path\to\franka_emika_panda\mjx_scene.xml")
```

---

## How to Run

Open and run the notebooks in order:

```bash
jupyter notebook
```

1. **`Forward_Kinematics_Analytically.ipynb`** — Run first to validate FK
2. **`Newton_Raphson_IK.ipynb`** — Run after FK to execute IK trajectory tracking

---

## Results

### FK Validation — 100 Random Configurations

| Metric | Position Error (m) | Rotation Error (rad) |
|---|---|---|
| Minimum | 6.24e-17 | 4.71e-08 |
| Mean | 6.01e-16 | 7.18e-08 |
| Maximum | 1.26e-15 | 8.94e-08 |

Errors are at machine precision, confirming the PoE implementation matches MuJoCo exactly.

### IK Trajectory Tracking — 5 Iterations per Waypoint

| Metric | Translational (m) | Rotational (rad) |
|---|---|---|
| RMSE | | |
| Mean | | |
| Std Dev | | |
| Maximum | | |

> Fill in your values from the trajectory error summary printout.

---

## Project Structure

```
.
├── Forward_Kinematics_Analytically.ipynb   # FK notebook
├── Newton_Raphson_IK.ipynb                 # IK notebook
├── franka_emika_panda/                     # MuJoCo model files
│   └── mjx_scene.xml
└── README.md
```

---

## References

- Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.
- Nakamura, Y., & Hanafusa, H. (1986). Inverse kinematic solutions with singularity robustness for robot manipulator control. *Journal of Dynamic Systems, Measurement, and Control*.
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Modern Robotics Python Library](https://github.com/NxRLab/ModernRobotics)

---

## Course

ME5250 — Robot Mechanics & Controls | Project 2
