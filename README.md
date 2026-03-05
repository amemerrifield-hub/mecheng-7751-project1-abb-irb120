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
