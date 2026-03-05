import { ABB_IRB120_FK, type JointVectorDeg, type Mat3 } from '../src/kinematics/abb120'

type RoboPose = { xyz: [number, number, number]; rzyxDeg: [number, number, number] }
type CaseData = { name: string; q: JointVectorDeg; robo: RoboPose }

const cases: CaseData[] = [
  { name: 'T1', q: [0, 0, 0, 0, 0, 0], robo: { xyz: [374, 0, 630], rzyxDeg: [0, 90, 0] } },
  { name: 'T2', q: [30, -45, 20, 40, -30, 60], robo: { xyz: [96.492, 28.99, 723.337], rzyxDeg: [112.051, -18.032, 41.412] } },
  { name: 'T3', q: [-60, 35, -40, -70, 45, -120], robo: { xyz: [209.494, -458.536, 594.316], rzyxDeg: [-52.419, -74.149, 131.03] } },
  { name: 'T4', q: [90, -80, 50, 100, -60, 200], robo: { xyz: [61.407, -2.768, 557.13], rzyxDeg: [-50.859, 50.287, -79.197] } },
  { name: 'T5', q: [-120, 70, -70, 140, 80, -300], robo: { xyz: [-244.638, -514.881, 506.663], rzyxDeg: [110.979, -38.549, 15.287] } },
  { name: 'T6', q: [150, -100, 65, -150, -100, 350], robo: { xyz: [11.445, -47.546, 416.203], rzyxDeg: [165.873, -30.404, -157.744] } },
]

const deg2rad = (d: number) => (d * Math.PI) / 180
const rad2deg = (r: number) => (r * 180) / Math.PI

function rotFromZYXDeg([rzDeg, ryDeg, rxDeg]: [number, number, number]): Mat3 {
  const rz = deg2rad(rzDeg)
  const ry = deg2rad(ryDeg)
  const rx = deg2rad(rxDeg)
  const cz = Math.cos(rz), sz = Math.sin(rz)
  const cy = Math.cos(ry), sy = Math.sin(ry)
  const cx = Math.cos(rx), sx = Math.sin(rx)
  return [
    [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
    [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
    [-sy, cy * sx, cy * cx],
  ]
}

function transpose3(m: Mat3): Mat3 {
  return [
    [m[0][0], m[1][0], m[2][0]],
    [m[0][1], m[1][1], m[2][1]],
    [m[0][2], m[1][2], m[2][2]],
  ]
}

function multiply3(a: Mat3, b: Mat3): Mat3 {
  const o: Mat3 = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) o[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c]
  return o
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, v))
}

function normalizeRoboEuler(r: [number, number, number]): [number, number, number] {
  return [r[0] - 180, -r[1], -r[2]]
}

for (const c of cases) {
  const T = ABB_IRB120_FK(c.q)
  const pFk: [number, number, number] = [T[0][3], T[1][3], T[2][3]]
  const RFk: Mat3 = [
    [T[0][0], T[0][1], T[0][2]],
    [T[1][0], T[1][1], T[1][2]],
    [T[2][0], T[2][1], T[2][2]],
  ]

  const pR = c.robo.xyz
  const dpx = pFk[0] - pR[0], dpy = pFk[1] - pR[1], dpz = pFk[2] - pR[2]
  const dp = Math.sqrt(dpx ** 2 + dpy ** 2 + dpz ** 2)

  const eNorm = normalizeRoboEuler(c.robo.rzyxDeg)
  const RR = rotFromZYXDeg(eNorm)
  const Rerr = multiply3(transpose3(RFk), RR)
  const tr = Rerr[0][0] + Rerr[1][1] + Rerr[2][2]
  const dR = rad2deg(Math.acos(clamp((tr - 1) / 2, -1, 1)))

  console.log(`${c.name}: dp=${dp.toFixed(6)} mm, dR=${dR.toFixed(6)} deg, normEuler=(${eNorm.map(v => v.toFixed(3)).join(', ')})`)
}
