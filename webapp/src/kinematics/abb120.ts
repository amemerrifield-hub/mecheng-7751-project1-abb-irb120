export type Mat3 = [number[], number[], number[]]
export type Mat4 = [number[], number[], number[], number[]]

export interface IkCandidate {
  jointsDeg: [number, number, number, number, number, number]
  posErrMm: number
  rotErrFro: number
}

export type JointVectorDeg = [number, number, number, number, number, number]

interface RobotParams {
  alpha: number[]
  a: number[]
  d: number[]
  offsetDeg: number[]
  jointLimitsDeg: {
    min: [number, number, number, number, number, number]
    max: [number, number, number, number, number, number]
  }
}

const EPS = 1e-9
const IK_POS_TOL_MM = 1.0
const IK_ROT_TOL_FRO = 0.01

function deg2rad(value: number): number {
  return (value * Math.PI) / 180
}

function rad2deg(value: number): number {
  return (value * 180) / Math.PI
}

function wrapTo180(value: number): number {
  return ((((value + 180) % 360) + 360) % 360) - 180
}

function wrapJointVector(values: number[]): [number, number, number, number, number, number] {
  return values.map((value) => wrapTo180(value)) as [number, number, number, number, number, number]
}

function identity4(): Mat4 {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ]
}

function multiply4(left: Mat4, right: Mat4): Mat4 {
  const result = identity4()
  for (let row = 0; row < 4; row += 1) {
    for (let col = 0; col < 4; col += 1) {
      let sum = 0
      for (let idx = 0; idx < 4; idx += 1) {
        sum += left[row][idx] * right[idx][col]
      }
      result[row][col] = sum
    }
  }
  return result
}

function copy4(matrix: Mat4): Mat4 {
  return [
    [...matrix[0]],
    [...matrix[1]],
    [...matrix[2]],
    [...matrix[3]],
  ]
}

function transpose3(matrix: Mat3): Mat3 {
  return [
    [matrix[0][0], matrix[1][0], matrix[2][0]],
    [matrix[0][1], matrix[1][1], matrix[2][1]],
    [matrix[0][2], matrix[1][2], matrix[2][2]],
  ]
}

function multiply3(left: Mat3, right: Mat3): Mat3 {
  const result: Mat3 = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ]
  for (let row = 0; row < 3; row += 1) {
    for (let col = 0; col < 3; col += 1) {
      let sum = 0
      for (let idx = 0; idx < 3; idx += 1) {
        sum += left[row][idx] * right[idx][col]
      }
      result[row][col] = sum
    }
  }
  return result
}

function frobeniusNorm3(matrix: Mat3): number {
  let sum = 0
  for (let row = 0; row < 3; row += 1) {
    for (let col = 0; col < 3; col += 1) {
      sum += matrix[row][col] * matrix[row][col]
    }
  }
  return Math.sqrt(sum)
}

function subtract3(left: Mat3, right: Mat3): Mat3 {
  return [
    [left[0][0] - right[0][0], left[0][1] - right[0][1], left[0][2] - right[0][2]],
    [left[1][0] - right[1][0], left[1][1] - right[1][1], left[1][2] - right[1][2]],
    [left[2][0] - right[2][0], left[2][1] - right[2][1], left[2][2] - right[2][2]],
  ]
}

function norm3(vector: [number, number, number]): number {
  return Math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
}

export function robotParams(): RobotParams {
  const alpha = [-90, 0, -90, 90, -90, 0].map((value) => deg2rad(value))
  const a = [0, 270, 70, 0, 0, 0]
  const d = [290, 0, 0, 302, 0, 72]
  const offsetDeg = [0, -90, 0, 0, 0, 0]

  return {
    alpha,
    a,
    d,
    offsetDeg,
    jointLimitsDeg: {
      min: [-165, -110, -90, -160, -120, -400],
      max: [165, 110, 70, 160, 120, 400],
    },
  }
}

export function ABB_IRB120_FK(qDeg: [number, number, number, number, number, number]): Mat4 {
  const params = robotParams()
  const q = qDeg.map((value, idx) => deg2rad(value + params.offsetDeg[idx]))

  let transform = identity4()
  for (let idx = 0; idx < 6; idx += 1) {
    const theta = q[idx]
    const alpha = params.alpha[idx]
    const ca = Math.cos(alpha)
    const sa = Math.sin(alpha)
    const ct = Math.cos(theta)
    const st = Math.sin(theta)

    const aMat: Mat4 = [
      [ct, -st * ca, st * sa, params.a[idx] * ct],
      [st, ct * ca, -ct * sa, params.a[idx] * st],
      [0, sa, ca, params.d[idx]],
      [0, 0, 0, 1],
    ]

    transform = multiply4(transform, aMat)
  }

  return transform
}

export function ABB_IRB120_FK_All(qDeg: JointVectorDeg): Mat4[] {
  const params = robotParams()
  const q = qDeg.map((value, idx) => deg2rad(value + params.offsetDeg[idx]))

  let transform = identity4()
  const transforms: Mat4[] = [copy4(transform)]
  for (let idx = 0; idx < 6; idx += 1) {
    const theta = q[idx]
    const alpha = params.alpha[idx]
    const ca = Math.cos(alpha)
    const sa = Math.sin(alpha)
    const ct = Math.cos(theta)
    const st = Math.sin(theta)

    const aMat: Mat4 = [
      [ct, -st * ca, st * sa, params.a[idx] * ct],
      [st, ct * ca, -ct * sa, params.a[idx] * st],
      [0, sa, ca, params.d[idx]],
      [0, 0, 0, 1],
    ]

    transform = multiply4(transform, aMat)
    transforms.push(copy4(transform))
  }

  return transforms
}

export function fkJointPositionsMm(qDeg: JointVectorDeg): [number, number, number][] {
  const frames = ABB_IRB120_FK_All(qDeg)
  return frames.map((frame) => [frame[0][3], frame[1][3], frame[2][3]])
}

export function ABB_IRB120_IK(targetT: Mat4): IkCandidate[] {
  const params = robotParams()
  const a2 = params.a[1]
  const a3 = params.a[2]
  const d1 = params.d[0]
  const d4 = params.d[3]
  const d6 = params.d[5]

  const targetR: Mat3 = [
    [targetT[0][0], targetT[0][1], targetT[0][2]],
    [targetT[1][0], targetT[1][1], targetT[1][2]],
    [targetT[2][0], targetT[2][1], targetT[2][2]],
  ]
  const targetP: [number, number, number] = [targetT[0][3], targetT[1][3], targetT[2][3]]

  const wristCenter: [number, number, number] = [
    targetP[0] - d6 * targetR[0][2],
    targetP[1] - d6 * targetR[1][2],
    targetP[2] - d6 * targetR[2][2],
  ]

  const px = wristCenter[0]
  const py = wristCenter[1]
  const pz = wristCenter[2]

  const q1a = Math.atan2(py, px)
  const q1b = q1a + Math.PI
  const q1List = [q1a, q1b]

  const r = Math.sqrt(px * px + py * py)
  const s = pz - d1

  const l2 = a2
  const l3 = Math.sqrt(a3 * a3 + d4 * d4)

  const D = (r * r + s * s - l2 * l2 - l3 * l3) / (2 * l2 * l3)
  if (Math.abs(D) > 1 + EPS) {
    return []
  }

  const clampedD = Math.max(-1, Math.min(1, D))
  const sqrtTerm = Math.sqrt(Math.max(0, 1 - clampedD * clampedD))
  const q3Options = [Math.atan2(sqrtTerm, clampedD), Math.atan2(-sqrtTerm, clampedD)]

  const candidates: IkCandidate[] = []

  for (const q1 of q1List) {
    for (const q3Prime of q3Options) {
      const phi = Math.atan2(s, r)
      const psi = Math.atan2(l3 * Math.sin(q3Prime), l2 + l3 * Math.cos(q3Prime))
      const q2Prime = -(phi + psi)

      const q1Deg = rad2deg(q1)
      const q2Deg = rad2deg(q2Prime) + 90
      const phiOff = Math.atan2(d4, a3)
      const q3Deg = rad2deg(q3Prime - phiOff)

      const t03 = ABB_IRB120_FK([q1Deg, q2Deg, q3Deg, 0, 0, 0])
      const r03: Mat3 = [
        [t03[0][0], t03[0][1], t03[0][2]],
        [t03[1][0], t03[1][1], t03[1][2]],
        [t03[2][0], t03[2][1], t03[2][2]],
      ]
      const r36 = multiply3(transpose3(r03), targetR)

      const q5 = Math.atan2(Math.sqrt(r36[0][2] ** 2 + r36[1][2] ** 2), r36[2][2])

      const wristSolutions: [number, number, number][] = []
      if (Math.abs(Math.sin(q5)) < 1e-6) {
        const q4 = 0
        const q6 = Math.atan2(r36[2][1], -r36[2][0])
        wristSolutions.push([q4, q5, q6])
      } else {
        const q4 = Math.atan2(-r36[1][2], -r36[0][2])
        const q6 = Math.atan2(-r36[2][1], r36[2][0])

        const q4b = q4 + Math.PI
        const q5b = -q5
        const q6b = q6 + Math.PI

        wristSolutions.push([q4, q5, q6])
        wristSolutions.push([q4b, q5b, q6b])
      }

      for (const wrist of wristSolutions) {
        const rawJointVector: [number, number, number, number, number, number] = [
          q1Deg,
          q2Deg,
          q3Deg,
          rad2deg(wrist[0]),
          rad2deg(wrist[1]),
          rad2deg(wrist[2]),
        ]

        const wrapped = wrapJointVector(rawJointVector)
        const checkedT = ABB_IRB120_FK(wrapped)
        const checkedR: Mat3 = [
          [checkedT[0][0], checkedT[0][1], checkedT[0][2]],
          [checkedT[1][0], checkedT[1][1], checkedT[1][2]],
          [checkedT[2][0], checkedT[2][1], checkedT[2][2]],
        ]

        const checkedP: [number, number, number] = [checkedT[0][3], checkedT[1][3], checkedT[2][3]]
        const posErrMm = norm3([
          checkedP[0] - targetP[0],
          checkedP[1] - targetP[1],
          checkedP[2] - targetP[2],
        ])
        const rotErrFro = frobeniusNorm3(subtract3(checkedR, targetR))

        candidates.push({
          jointsDeg: wrapped,
          posErrMm,
          rotErrFro,
        })
      }
    }
  }

  return candidates
}

export function rpyToTransformDeg(
  rpyDegZYX: [number, number, number],
  positionMm: [number, number, number],
): Mat4 {
  const rz = deg2rad(rpyDegZYX[0])
  const ry = deg2rad(rpyDegZYX[1])
  const rx = deg2rad(rpyDegZYX[2])

  const cz = Math.cos(rz)
  const sz = Math.sin(rz)
  const cy = Math.cos(ry)
  const sy = Math.sin(ry)
  const cx = Math.cos(rx)
  const sx = Math.sin(rx)

  const r00 = cz * cy
  const r01 = cz * sy * sx - sz * cx
  const r02 = cz * sy * cx + sz * sx
  const r10 = sz * cy
  const r11 = sz * sy * sx + cz * cx
  const r12 = sz * sy * cx - cz * sx
  const r20 = -sy
  const r21 = cy * sx
  const r22 = cy * cx

  return [
    [r00, r01, r02, positionMm[0]],
    [r10, r11, r12, positionMm[1]],
    [r20, r21, r22, positionMm[2]],
    [0, 0, 0, 1],
  ]
}

export function transformToRpyDegZYX(transform: Mat4): [number, number, number] {
  const r00 = transform[0][0]
  const r10 = transform[1][0]
  const r20 = transform[2][0]
  const r21 = transform[2][1]
  const r22 = transform[2][2]
  const r01 = transform[0][1]
  const r11 = transform[1][1]

  const cy = Math.sqrt(r00 * r00 + r10 * r10)
  const singular = cy < 1e-8

  let rz: number
  let ry: number
  let rx: number

  if (!singular) {
    rz = Math.atan2(r10, r00)
    ry = Math.atan2(-r20, cy)
    rx = Math.atan2(r21, r22)
  } else {
    rz = Math.atan2(-r01, r11)
    ry = Math.atan2(-r20, cy)
    rx = 0
  }

  return [rad2deg(rz), rad2deg(ry), rad2deg(rx)]
}

export function isIkCandidateValid(candidate: IkCandidate): boolean {
  return candidate.posErrMm <= IK_POS_TOL_MM && candidate.rotErrFro <= IK_ROT_TOL_FRO
}

export interface CartesianWorkspaceBounds {
  x: { min: number; max: number }
  y: { min: number; max: number }
  z: { min: number; max: number }
}

function lcg(seed: number): () => number {
  let state = seed >>> 0
  return () => {
    state = (1664525 * state + 1013904223) >>> 0
    return state / 4294967296
  }
}

export function estimateCartesianWorkspaceBoundsMm(sampleCount = 2500): CartesianWorkspaceBounds {
  const params = robotParams()
  const random = lcg(120)

  let minX = Number.POSITIVE_INFINITY
  let minY = Number.POSITIVE_INFINITY
  let minZ = Number.POSITIVE_INFINITY
  let maxX = Number.NEGATIVE_INFINITY
  let maxY = Number.NEGATIVE_INFINITY
  let maxZ = Number.NEGATIVE_INFINITY

  for (let idx = 0; idx < sampleCount; idx += 1) {
    const q: JointVectorDeg = [0, 0, 0, 0, 0, 0]
    for (let j = 0; j < 6; j += 1) {
      const min = params.jointLimitsDeg.min[j]
      const max = params.jointLimitsDeg.max[j]
      q[j] = min + (max - min) * random()
    }

    const t = ABB_IRB120_FK(q)
    const x = t[0][3]
    const y = t[1][3]
    const z = t[2][3]

    if (x < minX) minX = x
    if (y < minY) minY = y
    if (z < minZ) minZ = z
    if (x > maxX) maxX = x
    if (y > maxY) maxY = y
    if (z > maxZ) maxZ = z
  }

  const pad = 10
  return {
    x: { min: Math.floor(minX - pad), max: Math.ceil(maxX + pad) },
    y: { min: Math.floor(minY - pad), max: Math.ceil(maxY + pad) },
    z: { min: Math.floor(minZ - pad), max: Math.ceil(maxZ + pad) },
  }
}

function jointDistance(a: JointVectorDeg, b: JointVectorDeg): number {
  let sum = 0
  for (let idx = 0; idx < 6; idx += 1) {
    const delta = wrapTo180(b[idx] - a[idx])
    sum += delta * delta
  }
  return Math.sqrt(sum)
}

export function selectBestIkSolution(
  candidates: IkCandidate[],
  reference?: JointVectorDeg,
): IkCandidate | null {
  const validCandidates = candidates.filter((candidate) => isIkCandidateValid(candidate))
  if (validCandidates.length === 0) {
    return null
  }

  // Prefer candidates that lie within the robot's joint limits.
  const params = robotParams()
  const withinLimits = validCandidates.filter((candidate) =>
    candidate.jointsDeg.every((value, idx) => value >= params.jointLimitsDeg.min[idx] && value <= params.jointLimitsDeg.max[idx]),
  )

  if (withinLimits.length === 0) {
    // If no valid candidate is within limits, treat as no solution so the caller
    // can suggest a nearby reachable Cartesian point instead of silently
    // clamping joint values.
    return null
  }

  let best = withinLimits[0]
  let bestScore = Number.POSITIVE_INFINITY
  for (const candidate of withinLimits) {
    const errorScore = candidate.posErrMm * 5 + candidate.rotErrFro * 200
    const distanceScore = reference ? jointDistance(candidate.jointsDeg, reference) : 0
    const score = errorScore + distanceScore
    if (score < bestScore) {
      best = candidate
      bestScore = score
    }
  }

  return best
}

export function runRandomTests(count: number): {
  totalCandidates: number
  validCandidates: number
  successRate: number
} {
  const params = robotParams()
  const jointMin = params.jointLimitsDeg.min
  const jointMax = params.jointLimitsDeg.max

  let totalCandidates = 0
  let validCandidates = 0

  for (let idx = 0; idx < count; idx += 1) {
    const sample: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0]
    for (let joint = 0; joint < 6; joint += 1) {
      const min = jointMin[joint]
      const max = jointMax[joint]
      sample[joint] = min + (max - min) * Math.random()
    }

    const targetT = ABB_IRB120_FK(sample)
    const solutions = ABB_IRB120_IK(targetT)
    totalCandidates += solutions.length

    for (const solution of solutions) {
      if (solution.posErrMm < 1.0 && solution.rotErrFro < 0.01) {
        validCandidates += 1
      }
    }
  }

  return {
    totalCandidates,
    validCandidates,
    successRate: totalCandidates === 0 ? 0 : (100 * validCandidates) / totalCandidates,
  }
}
