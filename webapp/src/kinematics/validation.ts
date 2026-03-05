import {
  ABB_IRB120_FK,
  ABB_IRB120_IK,
  type JointVectorDeg,
  runRandomTests,
  selectBestIkSolution,
} from './abb120'

export interface ValidationCaseResult {
  name: string
  sourceQDeg: JointVectorDeg
  candidates: number
  bestPosErrMm: number
  bestRotErrFro: number
  recoveredQDeg: JointVectorDeg | null
}

export interface ValidationSummary {
  pass: boolean
  thresholds: {
    maxPosErrMm: number
    maxRotErrFro: number
    minRandomSuccessPct: number
  }
  deterministic: ValidationCaseResult[]
  random: {
    sampleCount: number
    solvedTargets: number
    successRatePct: number
  }
}

const deterministicCases: { name: string; q: JointVectorDeg }[] = [
  { name: 'Home-ish', q: [0, -30, 20, 0, 20, 0] },
  { name: 'Offset-1', q: [15, -40, 10, 20, -30, 45] },
  { name: 'Offset-2', q: [-60, 15, -20, 80, 35, -120] },
  { name: 'Wide-1', q: [90, -10, 30, -45, 60, 180] },
]

export function runValidationSuite(randomSampleCount = 200): ValidationSummary {
  const thresholds = {
    maxPosErrMm: 1.0,
    maxRotErrFro: 0.01,
    minRandomSuccessPct: 99,
  }

  const deterministic = deterministicCases.map((entry) => {
    const transform = ABB_IRB120_FK(entry.q)
    const candidates = ABB_IRB120_IK(transform)
    const best = selectBestIkSolution(candidates, entry.q)

    return {
      name: entry.name,
      sourceQDeg: entry.q,
      candidates: candidates.length,
      bestPosErrMm: best?.posErrMm ?? Number.POSITIVE_INFINITY,
      bestRotErrFro: best?.rotErrFro ?? Number.POSITIVE_INFINITY,
      recoveredQDeg: best?.jointsDeg ?? null,
    }
  })

  let solvedTargets = 0
  for (let i = 0; i < randomSampleCount; i += 1) {
    const one = runRandomTests(1)
    const hasValid = one.validCandidates > 0
    if (hasValid) {
      solvedTargets += 1
    }
  }

  const randomSuccessRate = (100 * solvedTargets) / randomSampleCount
  const deterministicPass = deterministic.every(
    (item) => item.bestPosErrMm <= thresholds.maxPosErrMm && item.bestRotErrFro <= thresholds.maxRotErrFro,
  )
  const randomPass = randomSuccessRate >= thresholds.minRandomSuccessPct

  return {
    pass: deterministicPass && randomPass,
    thresholds,
    deterministic,
    random: {
      sampleCount: randomSampleCount,
      solvedTargets,
      successRatePct: randomSuccessRate,
    },
  }
}
