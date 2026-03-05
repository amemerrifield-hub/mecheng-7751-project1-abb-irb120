# ABB IRB 120 Inverse Kinematics Derivation (Report Version)

## 1) Scope and Conventions
This report derives the analytical inverse kinematics (IK) used in `ABB_IRB120_FK_IK.m` for the ABB IRB 120 model in this project.

- Units: mm for position, degrees in user I/O, radians internally for trig.
- Target pose:
  \[
  T_{06}=\begin{bmatrix}R_{06} & p_{06}\\ \mathbf{0}_{1\times3} & 1\end{bmatrix},\quad R_{06}\in SO(3),\ p_{06}\in\mathbb{R}^3
  \]
- DH data (as used in code):
  - \(\alpha=[-90,\ 0,\ -90,\ 90,\ -90,\ 0]^\circ\)
  - \(a=[0,\ 270,\ 70,\ 0,\ 0,\ 0]\) mm
  - \(d=[290,\ 0,\ 0,\ 302,\ 0,\ 72]\) mm
  - Offsets: \(\theta_i=q_i+\text{offset}_i\), with offset \([0,-90,0,0,0,0]^\circ\)

## 2) Kinematic Decoupling
Because joints 4–6 form a spherical wrist, solve in two stages:

1. Position IK for \(q_1,q_2,q_3\) using wrist center.
2. Orientation IK for \(q_4,q_5,q_6\) from \(R_{36}\).

### 2.1 Wrist center
Let \(\hat z_6\) be the tool z-axis (third column of \(R_{06}\)). Then
\[
p_w = p_{06} - d_6\hat z_6 = p_{06} - d_6R_{06}(:,3)
\]
with \(d_6=72\) mm.

Define
\[
p_w=[p_x,\ p_y,\ p_z]^T,\quad r=\sqrt{p_x^2+p_y^2},\quad s=p_z-d_1,
\]
where \(d_1=290\) mm.

## 3) Solve \(q_1\): Shoulder Branches
Base rotation from planar projection:
\[
q_{1a}=\operatorname{atan2}(p_y,p_x),\qquad q_{1b}=q_{1a}+\pi.
\]
These are the two shoulder configurations (left/right around base axis).

## 4) Arm Triangle Geometry for \(q_2,q_3\)
Using this DH convention, the effective links in the arm plane are
\[
L_2=a_2=270,\qquad L_3=\sqrt{a_3^2+d_4^2}=\sqrt{70^2+302^2}.
\]

Distance from joint-2 origin to wrist center in the arm plane:
\[
\rho=\sqrt{r^2+s^2}.
\]

Apply law of cosines:
\[
D=\frac{\rho^2-L_2^2-L_3^2}{2L_2L_3}
=\frac{r^2+s^2-L_2^2-L_3^2}{2L_2L_3}.
\]
Reachability condition:
\[
|D|\le1.
\]
If \(|D|>1\), no real IK exists for the requested pose.

### 4.1 Elbow branches
Define geometric elbow angle \(q_3'\):
\[
q_3' = \operatorname{atan2}(\pm\sqrt{1-D^2},\ D)
\]
(\(+\) and \(-\) give elbow-up/down).

### 4.2 Shoulder angle in arm plane
\[
\phi = \operatorname{atan2}(s,r),\qquad
\psi = \operatorname{atan2}(L_3\sin q_3',\ L_2+L_3\cos q_3').
\]
For this frame orientation (matching code):
\[
q_2' = -(\phi+\psi).
\]

## 5) Map Geometric Angles to Robot Joint Variables
The geometric angles above are not yet the user joint values due to DH offsets and the \((a_3,d_4)\) composition.

### 5.1 Joint 2 offset
Since \(\theta_2=q_2-90^\circ\),
\[
q_2 = q_2' + 90^\circ.
\]

### 5.2 Joint 3 correction
Because \(L_3\) combines \(a_3\) and \(d_4\), define
\[
\phi_{off}=\operatorname{atan2}(d_4,a_3).
\]
Then
\[
q_3 = q_3' - \phi_{off}.
\]

So each arm branch yields one \((q_1,q_2,q_3)\) set.

## 6) Solve Wrist Orientation: \(q_4,q_5,q_6\)
Compute
\[
R_{36}=R_{03}^TR_{06},
\]
where \(R_{03}\) is obtained from FK with \([q_1,q_2,q_3,0,0,0]\).

Using the wrist convention in this project (\(\alpha_4=+90^\circ,\alpha_5=-90^\circ\)):
\[
q_5 = \operatorname{atan2}\left(\sqrt{R_{36}(1,3)^2+R_{36}(2,3)^2},\ R_{36}(3,3)\right).
\]

If \(|\sin q_5|\approx0\) (wrist singularity):
- Set \(q_4=0\)
- Solve
  \[
  q_6=\operatorname{atan2}(R_{36}(3,2),-R_{36}(3,1)).
  \]

Otherwise principal wrist solution:
\[
q_4=\operatorname{atan2}(-R_{36}(2,3),-R_{36}(1,3)),\quad
q_6=\operatorname{atan2}(-R_{36}(3,2),R_{36}(3,1)).
\]
Second wrist-flip branch:
\[
(q_4,q_5,q_6)\rightarrow(q_4+\pi,-q_5,q_6+\pi).
\]

## 7) Number of IK Branches
Nominally:
\[
2\ (q_1)\times2\ (q_3')\times2\ (\text{wrist})=8
\]
candidate solutions.

Each candidate is wrapped to \([-180^\circ,180^\circ]\), then validated by FK:
\[
e_p=\|p_{FK}-p_{target}\|,
\qquad
e_R=\|R_{FK}-R_{target}\|_F.
\]
The implementation returns all candidates plus \((e_p,e_R)\) for ranking/filtering.

## 8) Algorithm Flow (Implementation-Aligned)
```mermaid
flowchart TD
    A[Input target T06] --> B[Extract R06 and p06]
    B --> C[Compute wrist center pw = p06 - d6*R06(:,3)]
    C --> D[Compute q1 branches]
    D --> E[Compute D from law of cosines]
    E -->|abs(D)>1| F[No real IK]
    E -->|abs(D)<=1| G[Two q3' branches]
    G --> H[Compute q2' from phi and psi]
    H --> I[Map to q2 and q3 with offsets]
    I --> J[Compute R03 and R36 = R03^T*R06]
    J --> K[Extract q4 q5 q6 and wrist-flip branch]
    K --> L[Wrap angles and FK-check errors]
    L --> M[Return all candidates]
```

## 9) Notes for Report/Defense
- The main derivation choice is the geometric reduction \((a_3,d_4)\to L_3\), which explains the \(\phi_{off}=\operatorname{atan2}(d_4,a_3)\) correction in \(q_3\).
- The sign in \(q_2'=-(\phi+\psi)\) is convention-dependent and must match the adopted DH frame directions.
- Wrist singularity handling removes one DOF; setting \(q_4=0\) is a practical gauge choice.

---
This document is intentionally aligned with the exact equations and branching logic implemented in `ABB_IRB120_FK_IK.m`.
