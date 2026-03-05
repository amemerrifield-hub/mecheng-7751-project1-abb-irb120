# Project 1 Brief Report — ABB IRB120 Kinematics

## 1) Scope

This project implements and validates:

1. Forward Kinematics (FK) for ABB IRB120 using DH parameters.
2. Inverse Kinematics (IK) for 6-DOF pose recovery with multiple solution branches.
3. A web-based motion planning/visualization app.

## 2) Kinematic model and derivation summary

- Standard homogeneous transforms are used for each joint:

$$
{}^{i-1}T_i =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

- End-effector transform:

$$
{}^0T_6 = {}^0T_1 {}^1T_2 {}^2T_3 {}^3T_4 {}^4T_5 {}^5T_6
$$

- MATLAB FK function now outputs both:
  - individual link transforms `A_i`
  - cumulative transforms `T_0i`

## 3) IK approach summary

- Wrist center computed from target pose:

$$
p_w = p_6 - d_6 \hat{z}_6
$$

- Arm solution (`q1,q2,q3`) solved geometrically using planar triangle relations.
- Wrist orientation (`q4,q5,q6`) solved from $R_{36}$ with branch handling and wrist-flip solutions.
- Multiple candidate solutions are generated and validated through FK reconstruction error checks.

## 4) Implementation notes

- MATLAB implementation files are in `Matlab/`.
- Web implementation files are in `webapp/src/kinematics/`.
- Joint wrapping and branch management are used to keep angle solutions consistent.

## 5) Validation and test summary

- Home-pose FK sanity checks completed.
- FK→IK→FK round-trip checks used for solution validation.
- Randomized test scripts included:
  - `run_20_tests.m`
  - `verify_ik_correct.m`

Observed status from current test notes:

- IK wrist extraction has been corrected.
- Most random test cases pass; remaining failures are near singular or edge-workspace poses.

## 6) Design decisions

- Kept FK/IK in both MATLAB and TypeScript for reproducibility and interactive visualization.
- Used explicit matrix-based DH formulation for traceability in report derivations.
- Preserved multi-branch IK output rather than forcing a single answer.

## 7) Limitations and future improvements

- Add stronger singularity handling near $\sin(q_5) \approx 0$.
- Add deterministic solution ranking by joint-limit margin and continuity from previous state.
- Expand automated tests around workspace boundaries and near-singular poses.

## 8) Reproducibility

- MATLAB: run scripts in `Matlab/`.
- Web app: see `webapp/README.md` for run/build/deploy instructions.
- Hosted web app (GitHub Pages):
  - `https://amemerrifield-hub.github.io/mecheng-7751-project1-abb-irb120/`

## 9) AI tool usage disclosure

- GitHub Copilot (GPT-5.3-Codex) and Codex-assisted workflows were used during development of both:
  - the analytical IK implementation and related debugging scripts in MATLAB
  - the React + TypeScript web app implementation and deployment configuration
- All generated code and text were reviewed, edited, and validated through FK/IK round-trip tests, random-case checks, and manual inspection before inclusion in the final project materials.
