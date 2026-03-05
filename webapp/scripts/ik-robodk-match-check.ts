import { ABB_IRB120_FK, ABB_IRB120_IK, type JointVectorDeg } from '../src/kinematics/abb120'

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

function wrap180(v: number): number {
  return ((((v + 180) % 360) + 360) % 360) - 180
}

function wrapVec(values: number[]): number[] {
  return values.map((v) => wrap180(v))
}

function maxJointDiffDeg(a: number[], b: number[]): number {
  let max = 0
  for (let i = 0; i < 6; i += 1) {
    const diff = Math.abs(wrap180(a[i] - b[i]))
    if (diff > max) max = diff
  }
  return max
}

const tolDeg = 1.0

for (const test of cases) {
  const T = ABB_IRB120_FK(test.qTest)
  const analytical = ABB_IRB120_IK(T).map((c) => c.jointsDeg)

  const analyticalWrapped = analytical.map((v) => wrapVec(v))
  const roboWrapped = test.roboSolutions.map((v) => wrapVec(v))

  const matches = roboWrapped.map((r) => {
    let best = Number.POSITIVE_INFINITY
    let bestIdx = -1
    analyticalWrapped.forEach((a, idx) => {
      const d = maxJointDiffDeg(r, a)
      if (d < best) {
        best = d
        bestIdx = idx
      }
    })
    return { robo: r, bestDiff: best, bestIdx, match: best <= tolDeg }
  })

  const matchedCount = matches.filter((m) => m.match).length

  console.log(`\n${test.caseId}:`) 
  console.log(`  Analytical branches: ${analyticalWrapped.length}`)
  console.log(`  RoboDK solutions (raw): ${test.roboSolutions.length}`)
  console.log(`  Matches within ${tolDeg.toFixed(1)} deg: ${matchedCount}/${test.roboSolutions.length}`)

  matches.forEach((m, i) => {
    console.log(
      `  - Robo #${i + 1} -> best analytical #${m.bestIdx + 1}, max joint diff=${m.bestDiff.toFixed(3)} deg, ${m.match ? 'MATCH' : 'NO MATCH'}`,
    )
  })

  const uniq = new Set(roboWrapped.map((v) => v.map((x) => x.toFixed(1)).join(',')))
  console.log(`  RoboDK unique solutions after wrapping: ${uniq.size}`)
}
