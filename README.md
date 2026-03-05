# MECHENG 7751 Project 1 — ABB IRB120 Kinematics

This repository contains the full source code and documentation for Project 1:

- Forward Kinematics (FK)
- Inverse Kinematics (IK)
- Motion planning and visualization web application

## Repository structure

- `.github/`
  - GitHub Actions workflows used to automate deployment tasks (for example, publishing the web app).
- `.vscode/`
  - Local VS Code workspace settings and editor configuration for this project.
- `figures/`
  - Images used in reports, slides, and validation comparisons (for example, RoboDK and app screenshots).
- `Matlab/`
  - MATLAB implementations for FK/IK, diagnostics, and validation scripts.
  - Primary files:
    - `ABB_IRB120_FK_IK.m`
    - `ABB_IRB120_FK.m`
    - `run_20_tests.m`
    - `verify_ik_correct.m`
- `presentation and report/`
  - Final presentation deck and written report artifacts (`.pptx`, `.pdf`, `.tex`, `.md`).
- `scripts/`
  - Utility scripts for repository workflow tasks (such as Windows PowerShell helper scripts).
- `setup and problem statement/`
  - Source materials and reference files from the assignment setup (spec sheet, DH references, and robot model files).
- `webapp/`
  - React + TypeScript app for interactive kinematics/motion planning.
  - Includes FK/IK implementation in `webapp/src/kinematics/abb120.ts`.
- `README.md`
  - Project overview and instructions for MATLAB and web app workflows.

## MATLAB file notes

- `ABB_IRB120_FK_IK.m` is the main MATLAB workflow file.
  - Provides an interactive menu for FK/IK demos and random testing.
  - Supports programmatic modes so the same file can be used like a function:
    - `ABB_IRB120_FK_IK("fk", q)`
      - Input: `q` is a `1x6` joint vector in degrees.
      - Output: `T` (`4x4` homogeneous transform of the end effector).
    - `ABB_IRB120_FK_IK("ik", T)` or `ABB_IRB120_FK_IK("ik", T, debug)`
      - Input: `T` is a `4x4` target pose.
      - Output: `sols` (`N x 8` matrix) where columns are:
        - `1:6` = joint solution `[q1 q2 q3 q4 q5 q6]` in degrees
        - `7` = position error in mm (`pos_err_mm`)
        - `8` = rotation error (Frobenius norm, `R_err_fro`)
      - Up to 8 branch candidates can be returned (shoulder/elbow/wrist branches).
    - `ABB_IRB120_FK_IK("test", n)`
      - Runs `n` random FK->IK validation tests inside joint limits.
      - Output is printed diagnostics (per-test candidate quality and overall success rate).
  - In interactive mode (calling `ABB_IRB120_FK_IK` with no arguments), menu options 2-4 print:
    - the target/forward transform `T`
    - a formatted IK solution table with validity status based on error thresholds.
  - Contains the analytical IK implementation and helper routines in one self-contained file.
- `ABB_IRB120_FK.m` is a standalone FK implementation used for clear transform outputs and verification work.
- Validation scripts such as `run_20_tests.m`, `verify_ik_correct.m`, and `test_ik_fix.m` are included for IK/FK accuracy checks.

## MATLAB usage

Open MATLAB in the `Matlab/` directory and run, for example:

```matlab
ABB_IRB120_FK
```

or

```matlab
run_20_tests
```

Notes:

- `ABB_IRB120_FK.m` returns:
  - `T`: end-effector transform
  - `A_all`: individual DH matrices `A_i`
  - `T_all`: cumulative transforms `T_0i` (useful for report tables)

- To use the combined FK/IK demo workflow:

```matlab
ABB_IRB120_FK_IK
```

- To call it directly in script/function style:

```matlab
T = ABB_IRB120_FK_IK("fk", [0 -30 30 0 0 0]);
sols = ABB_IRB120_FK_IK("ik", T);
ABB_IRB120_FK_IK("test", 20);
```

## Web app usage

From `webapp/`:

```bash
npm install
npm run dev
```

Build and preview:

```bash
npm run build
npm run preview
```

Run the webapp locally

Prerequisites:
- Node.js >= 20.19.0 (use `nvm` or `nvm-windows` to manage versions)
- `npm` (bundled with Node)

From the `webapp/` folder:

```powershell
nvm use 20.19.0
cd webapp
npm install
npm run dev
```

Vite typically serves at http://localhost:5173 — open that URL in a browser.

If you want to commit and push local helper scripts, `git` must be available in PATH. The `scripts/push_change.ps1` helper automates commit/pull/push on Windows.

## Web app online hosting

The web app is hosted on GitHub Pages for permanent online access:

`https://amemerrifield-hub.github.io/mecheng-7751-project1-abb-irb120/`

Deployment is automated by GitHub Actions using `.github/workflows/deploy-webapp-gh-pages.yml`.

If the latest changes are not visible yet, open the repository Actions tab and confirm the most recent `Deploy Webapp to GitHub Pages` run has completed successfully.
