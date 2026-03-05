# ABB IRB120 Motion Planning Web App

This folder contains the React + TypeScript app used to visualize ABB IRB120 kinematics and run motion-planning demos for MECHENG 7751 Project 1.

## Prerequisites

- Node.js 20+
- npm 10+

## Install

```bash
npm install
```

If PowerShell blocks `npm` scripts, use `npm.cmd` instead:

```powershell
npm.cmd install
```

## Run locally

```bash
npm run dev
```

PowerShell fallback:

```powershell
npm.cmd run dev
```

Open the local URL shown in the terminal (typically `http://localhost:5173`).

## Build

```bash
npm run build
```

PowerShell fallback:

```powershell
npm.cmd run build
```

## Preview production build

```bash
npm run preview
```

## Validate FK/IK implementation

```bash
npm run validate:kinematics
```

## Deploy to GitHub Pages

1. Create a GitHub repository and push this project.
2. In this `webapp/` folder, run:

```bash
npm run deploy
```

PowerShell fallback:

```powershell
npm.cmd run deploy
```

3. In GitHub, open **Settings → Pages** and ensure the source is set to **gh-pages branch**.
4. Your site will be published at a URL of the form:

```text
https://<username>.github.io/<repo-name>/
```
