import { ABB_IRB120_FK, ABB_IRB120_IK, type JointVectorDeg, type Mat3 } from '../src/kinematics/abb120'

type CaseRow = {
  id: string
  q: JointVectorDeg
}

const cases: CaseRow[] = [
  { id: 'C1', q: [0, 0, 0, 0, 0, 0] },
  { id: 'C2', q: [30, -45, 20, 40, -30, 60] },
  { id: 'C3', q: [-60, 35, -40, -70, 45, -120] },
  { id: 'C4', q: [150, -100, 65, -150, -100, 350] },
  { id: 'C5', q: [0, -110, -90, 0, 0, 0] },
  { id: 'C6', q: [0, 110, 70, 0, 0, 0] },
  { id: 'C7', q: [45, 0, 0, 90, 0, -90] },
  { id: 'C8', q: [-120, 70, -70, 140, 80, -300] },
]

function rad2deg(r: number): number { return (r * 180) / Math.PI }

function zyxDegFromRot(R: Mat3): [number, number, number] {
  const sy = -R[2][0]
  const cy = Math.sqrt(Math.max(0, 1 - sy * sy))
  let rz: number
  let ry: number
  let rx: number
  ry = Math.atan2(sy, cy)
  if (cy > 1e-9) {
    rz = Math.atan2(R[1][0], R[0][0])
    rx = Math.atan2(R[2][1], R[2][2])
  } else {
    rz = Math.atan2(-R[0][1], R[1][1])
    rx = 0
  }
  return [rad2deg(rz), rad2deg(ry), rad2deg(rx)]
}

for (const c of cases) {
  const T = ABB_IRB120_FK(c.q)
  const p = [T[0][3], T[1][3], T[2][3]]
  const R: Mat3 = [
    [T[0][0], T[0][1], T[0][2]],
    [T[1][0], T[1][1], T[1][2]],
    [T[2][0], T[2][1], T[2][2]],
  ]
  const e = zyxDegFromRot(R)
  const sols = ABB_IRB120_IK(T)
  let bestPos = Number.POSITIVE_INFINITY
  let bestRot = Number.POSITIVE_INFINITY
  for (const s of sols) {
    if (s.posErrMm < bestPos) bestPos = s.posErrMm
    if (s.rotErrFro < bestRot) bestRot = s.rotErrFro
  }

  console.log(
    `${c.id}|q=[${c.q.join(', ')}]|pose=(${p.map((v) => v.toFixed(3)).join(', ')}, ${e
      .map((v) => v.toFixed(3))
      .join(', ')})|N=${sols.length}|best=(${bestPos.toFixed(6)} mm, ${bestRot.toExponential(3)})`,
  )
}
