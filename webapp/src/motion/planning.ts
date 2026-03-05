import {
  ABB_IRB120_FK,
  ABB_IRB120_IK,
  type JointVectorDeg,
  type Mat3,
  type Mat4,
  robotParams,
  selectBestIkSolution,
} from '../kinematics/abb120'

export interface TimedJointState {
  t: number
  q: JointVectorDeg
}

export interface PlannedTrajectory {
  kind: 'MoveJ' | 'MoveL'
  samples: TimedJointState[]
}

export type TimeProfile = 'cubic' | 'lspb'

function lerp(a: number, b: number, s: number): number {
  return a + (b - a) * s
}

function smoothstep01(s: number): number {
  const c = Math.max(0, Math.min(1, s))
  return c * c * (3 - 2 * c)
}

function lspbScale01(s: number, accelFraction = 0.2): number {
  const u = Math.max(0, Math.min(1, s))
  const r = Math.max(0.05, Math.min(0.45, accelFraction))

  if (u < r) {
    return (u * u) / (2 * r * (1 - r))
  }

  if (u <= 1 - r) {
    return (u - r / 2) / (1 - r)
  }

  const d = 1 - u
  return 1 - (d * d) / (2 * r * (1 - r))
}

function timeScale(s: number, profile: TimeProfile): number {
  if (profile === 'lspb') {
    return lspbScale01(s)
  }
  return smoothstep01(s)
}

function wrapDeltaDeg(from: number, to: number): number {
  return ((((to - from + 180) % 360) + 360) % 360) - 180
}

function matrixToRotation(transform: Mat4): Mat3 {
  return [
    [transform[0][0], transform[0][1], transform[0][2]],
    [transform[1][0], transform[1][1], transform[1][2]],
    [transform[2][0], transform[2][1], transform[2][2]],
  ]
}

function multiply3(left: Mat3, right: Mat3): Mat3 {
  const out: Mat3 = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ]
  for (let row = 0; row < 3; row += 1) {
    for (let col = 0; col < 3; col += 1) {
      out[row][col] =
        left[row][0] * right[0][col] + left[row][1] * right[1][col] + left[row][2] * right[2][col]
    }
  }
  return out
}

function transpose3(matrix: Mat3): Mat3 {
  return [
    [matrix[0][0], matrix[1][0], matrix[2][0]],
    [matrix[0][1], matrix[1][1], matrix[2][1]],
    [matrix[0][2], matrix[1][2], matrix[2][2]],
  ]
}

function skew(axis: [number, number, number]): Mat3 {
  return [
    [0, -axis[2], axis[1]],
    [axis[2], 0, -axis[0]],
    [-axis[1], axis[0], 0],
  ]
}

function identity3(): Mat3 {
  return [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ]
}

function add3(a: Mat3, b: Mat3): Mat3 {
  return [
    [a[0][0] + b[0][0], a[0][1] + b[0][1], a[0][2] + b[0][2]],
    [a[1][0] + b[1][0], a[1][1] + b[1][1], a[1][2] + b[1][2]],
    [a[2][0] + b[2][0], a[2][1] + b[2][1], a[2][2] + b[2][2]],
  ]
}

function scale3(matrix: Mat3, scalar: number): Mat3 {
  return [
    [matrix[0][0] * scalar, matrix[0][1] * scalar, matrix[0][2] * scalar],
    [matrix[1][0] * scalar, matrix[1][1] * scalar, matrix[1][2] * scalar],
    [matrix[2][0] * scalar, matrix[2][1] * scalar, matrix[2][2] * scalar],
  ]
}

function axisAngleFromRotation(rotation: Mat3): { axis: [number, number, number]; angle: number } {
  const tr = rotation[0][0] + rotation[1][1] + rotation[2][2]
  const cosTheta = Math.max(-1, Math.min(1, (tr - 1) / 2))
  const angle = Math.acos(cosTheta)

  if (Math.abs(angle) < 1e-9) {
    return { axis: [1, 0, 0], angle: 0 }
  }

  const denom = 2 * Math.sin(angle)
  const axis: [number, number, number] = [
    (rotation[2][1] - rotation[1][2]) / denom,
    (rotation[0][2] - rotation[2][0]) / denom,
    (rotation[1][0] - rotation[0][1]) / denom,
  ]

  const norm = Math.hypot(axis[0], axis[1], axis[2])
  if (norm < 1e-9) {
    return { axis: [1, 0, 0], angle }
  }
  return { axis: [axis[0] / norm, axis[1] / norm, axis[2] / norm], angle }
}

function rotationExp(axis: [number, number, number], angle: number): Mat3 {
  if (Math.abs(angle) < 1e-9) {
    return identity3()
  }

  const hat = skew(axis)
  const hat2 = multiply3(hat, hat)
  return add3(add3(identity3(), scale3(hat, Math.sin(angle))), scale3(hat2, 1 - Math.cos(angle)))
}

function composeTransform(rotation: Mat3, position: [number, number, number]): Mat4 {
  return [
    [rotation[0][0], rotation[0][1], rotation[0][2], position[0]],
    [rotation[1][0], rotation[1][1], rotation[1][2], position[1]],
    [rotation[2][0], rotation[2][1], rotation[2][2], position[2]],
    [0, 0, 0, 1],
  ]
}

export function planMoveJ(
  qStart: JointVectorDeg,
  qGoal: JointVectorDeg,
  steps = 80,
  durationSeconds = 4,
  profile: TimeProfile = 'lspb',
): PlannedTrajectory {
  const sampleCount = Math.max(2, steps)
  const samples: TimedJointState[] = []

  for (let idx = 0; idx < sampleCount; idx += 1) {
    const s = idx / (sampleCount - 1)
    const sigma = timeScale(s, profile)
    const q = qStart.map((startValue, jointIdx) => startValue + wrapDeltaDeg(startValue, qGoal[jointIdx]) * sigma) as JointVectorDeg
    samples.push({ t: s * durationSeconds, q })
  }

  return { kind: 'MoveJ', samples }
}

export function planMoveL(
  qStart: JointVectorDeg,
  qGoal: JointVectorDeg,
  steps = 80,
  durationSeconds = 4,
  profile: TimeProfile = 'lspb',
): PlannedTrajectory {
  const limits = robotParams().jointLimitsDeg
  const startT = ABB_IRB120_FK(qStart)
  const goalT = ABB_IRB120_FK(qGoal)

  const p0: [number, number, number] = [startT[0][3], startT[1][3], startT[2][3]]
  const p1: [number, number, number] = [goalT[0][3], goalT[1][3], goalT[2][3]]

  const r0 = matrixToRotation(startT)
  const r1 = matrixToRotation(goalT)
  const rRel = multiply3(transpose3(r0), r1)
  const { axis, angle } = axisAngleFromRotation(rRel)

  const sampleCount = Math.max(2, steps)
  const samples: TimedJointState[] = []

  let previous = qStart
  for (let idx = 0; idx < sampleCount; idx += 1) {
    const s = idx / (sampleCount - 1)
    const sigma = timeScale(s, profile)

    const p: [number, number, number] = [
      lerp(p0[0], p1[0], sigma),
      lerp(p0[1], p1[1], sigma),
      lerp(p0[2], p1[2], sigma),
    ]

    const rInterp = multiply3(r0, rotationExp(axis, angle * sigma))
    const target = composeTransform(rInterp, p)

    const candidates = ABB_IRB120_IK(target)
    const best = selectBestIkSolution(candidates, previous)
    if (!best) {
      throw new Error(`MoveL failed at sample ${idx + 1}/${sampleCount}: IK has no solution.`)
    }

    const hasNonFinite = best.jointsDeg.some((value) => !Number.isFinite(value))
    if (hasNonFinite) {
      throw new Error(`MoveL failed at sample ${idx + 1}/${sampleCount}: non-finite joint value.`)
    }

    const outsideLimits = best.jointsDeg.some((value, jointIdx) => value < limits.min[jointIdx] || value > limits.max[jointIdx])
    if (outsideLimits) {
      throw new Error(`MoveL failed at sample ${idx + 1}/${sampleCount}: IK solution outside joint limits.`)
    }

    previous = best.jointsDeg
    samples.push({ t: s * durationSeconds, q: best.jointsDeg })
  }

  return { kind: 'MoveL', samples }
}
