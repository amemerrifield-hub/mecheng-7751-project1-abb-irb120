import { ABB_IRB120_FK, type JointVectorDeg, type Mat3 } from '../src/kinematics/abb120'

type CaseInput = {
  caseId: 'C2' | 'C4' | 'C5' | 'C7'
  qTest: JointVectorDeg
  roboSolutions: number[][]
}

const cases: CaseInput[] = [
  {
    caseId: 'C2',
    qTest: [30, -45, 20, 40, -30, 60],
    roboSolutions: [
      [30, -45, 20, 40, -30, 60],
      [30, -45, 20, -140, 30, -120],
      [-150, -60.9, 20, -18.7, -90.3, -84.1],
      [30, -45, 20, -140, 30, 240],
      [30, -45, 20, 40, -30, -300],
      [-150, -60.9, 20, -18.7, -90.3, 275.9],
    ],
  },
  {
    caseId: 'C4',
    qTest: [150, -100, 65, -150, -100, 350],
    roboSolutions: [
      [150, -100, 65, -150, -100, 350],
      [-30, -64.6, 65, 148.4, -109.9, 152.4],
      [150, -100, 65, 30, 100, 170],
      [150, -100, 65, -150, -100, -10],
      [-30, -64.6, 65, -31.6, 109.9, 332.4],
      [-30, -64.6, 65, -31.6, 109.9, -387.6],
      [-30, -64.6, 65, 148.4, -109.9, -207.6],
      [150, -100, 65, 30, 100, -190],
      [-30, -64.6, 65, -31.6, 109.9, -27.6],
      [150, -100, 65, -150, -100, -370],
    ],
  },
  {
    caseId: 'C5',
    qTest: [0, -110, -90, 0, 0, 0],
    roboSolutions: [
      [0, -110, -90, 0, 0, 0],
      [0, -110, -90, 0, 0, 360],
      [0, -110, -90, 0, 0, -360],
    ],
  },
  {
    caseId: 'C7',
    qTest: [45, 0, 0, 90, 0, -90],
    roboSolutions: [
      [45, 0, 0, 90, 0, -90],
      [-135, -83.2, 0, 0, -96.8, 180],
      [45, 0, 0, 90, 0, 270],
      [-135, -83.2, 0, 0, -96.8, -180],
    ],
  },
]

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

function rad2deg(r: number): number { return (r * 180) / Math.PI }

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, v))
}

for (const c of cases) {
  const Ttarget = ABB_IRB120_FK(c.qTest)
  const pT = [Ttarget[0][3], Ttarget[1][3], Ttarget[2][3]]
  const RT: Mat3 = [
    [Ttarget[0][0], Ttarget[0][1], Ttarget[0][2]],
    [Ttarget[1][0], Ttarget[1][1], Ttarget[1][2]],
    [Ttarget[2][0], Ttarget[2][1], Ttarget[2][2]],
  ]

  console.log(`\n${c.caseId}:`) 
  c.roboSolutions.forEach((q, idx) => {
    const T = ABB_IRB120_FK(q as JointVectorDeg)
    const p = [T[0][3], T[1][3], T[2][3]]
    const R: Mat3 = [
      [T[0][0], T[0][1], T[0][2]],
      [T[1][0], T[1][1], T[1][2]],
      [T[2][0], T[2][1], T[2][2]],
    ]

    const dp = Math.sqrt((p[0]-pT[0])**2 + (p[1]-pT[1])**2 + (p[2]-pT[2])**2)
    const Rerr = multiply3(transpose3(R), RT)
    const tr = Rerr[0][0] + Rerr[1][1] + Rerr[2][2]
    const dR = rad2deg(Math.acos(clamp((tr - 1) / 2, -1, 1)))

    console.log(`  Robo #${idx + 1}: dp=${dp.toFixed(3)} mm, dR=${dR.toFixed(3)} deg`)
  })
}
