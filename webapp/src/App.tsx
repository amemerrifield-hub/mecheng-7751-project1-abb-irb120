import { useCallback, useEffect, useMemo, useState } from 'react'
import {
  ABB_IRB120_FK,
  ABB_IRB120_IK,
  estimateCartesianWorkspaceBoundsMm,
  type JointVectorDeg,
  robotParams,
  rpyToTransformDeg,
  selectBestIkSolution,
  transformToRpyDegZYX,
} from './kinematics/abb120'
import { runValidationSuite, type ValidationSummary } from './kinematics/validation'
import { JointChart } from './components/JointChart'
import { RobotViewer } from './components/RobotViewer'
import { planMoveJ, planMoveL, type PlannedTrajectory, type TimeProfile } from './motion/planning'
import './App.css'

type WaypointMode = 'joint' | 'cartesian'

type CartesianPose = {
  position: [number, number, number]
  rpyZYX: [number, number, number]
}

type SuggestedCartesianPoint = {
  which: 'start' | 'goal'
  position: [number, number, number]
  distanceMm: number
}

const defaultStartJ: JointVectorDeg = [0, 0, 0, 0, 0, 0]
const defaultGoalJ: JointVectorDeg = [45, 10, -20, 30, -25, 40]

function jointToCartesianPose(joints: JointVectorDeg): CartesianPose {
  const transform = ABB_IRB120_FK(joints)
  const rpy = transformToRpyDegZYX(transform)
  return {
    position: [
      roundTo(transform[0][3], 2),
      roundTo(transform[1][3], 2),
      roundTo(transform[2][3], 2),
    ],
    rpyZYX: [
      roundTo(rpy[0], 2),
      roundTo(rpy[1], 2),
      roundTo(rpy[2], 2),
    ],
  }
}

function roundTo(value: number, decimals: number): number {
  const scale = 10 ** decimals
  return Math.round(value * scale) / scale
}

function formatTriplet(values: [number, number, number], decimals = 2): string {
  return values.map((value) => roundTo(value, decimals).toFixed(decimals)).join(', ')
}

function normalizeDirection(direction: [number, number, number]): [number, number, number] {
  const norm = Math.hypot(direction[0], direction[1], direction[2])
  if (norm < 1e-9) {
    return [1, 0, 0]
  }
  return [direction[0] / norm, direction[1] / norm, direction[2] / norm]
}

function cartesianSearchDirections(): [number, number, number][] {
  const raw: [number, number, number][] = [
    [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
    [1, 1, 0], [1, -1, 0], [-1, 1, 0], [-1, -1, 0],
    [1, 0, 1], [1, 0, -1], [-1, 0, 1], [-1, 0, -1],
    [0, 1, 1], [0, 1, -1], [0, -1, 1], [0, -1, -1],
    [1, 1, 1], [1, 1, -1], [1, -1, 1], [1, -1, -1],
    [-1, 1, 1], [-1, 1, -1], [-1, -1, 1], [-1, -1, -1],
  ]

  return raw.map((direction) => normalizeDirection(direction))
}

const nearestSearchDirections = cartesianSearchDirections()

function findNearestReachableCartesianPosition(
  pose: CartesianPose,
  reference?: JointVectorDeg,
): { position: [number, number, number]; joints: JointVectorDeg; distanceMm: number } | null {
  const maxRadiusMm = 500
  const stepMm = 10

  for (let radius = stepMm; radius <= maxRadiusMm; radius += stepMm) {
    for (const direction of nearestSearchDirections) {
      const candidatePosition: [number, number, number] = [
        pose.position[0] + direction[0] * radius,
        pose.position[1] + direction[1] * radius,
        pose.position[2] + direction[2] * radius,
      ]

      const transform = rpyToTransformDeg(pose.rpyZYX, candidatePosition)
      const solutions = ABB_IRB120_IK(transform)
      const best = selectBestIkSolution(solutions, reference)
      if (best) {
        return {
          position: candidatePosition,
          joints: clampJointToLimits(best.jointsDeg),
          distanceMm: radius,
        }
      }
    }
  }

  return null
}

function resolveJointFromPose(
  pose: CartesianPose,
  reference?: JointVectorDeg,
): {
  joints: JointVectorDeg | null
  suggestion: SuggestedCartesianPoint | null
  message?: string
} {
  const transform = rpyToTransformDeg(pose.rpyZYX, pose.position)
  const solutions = ABB_IRB120_IK(transform)
  const best = selectBestIkSolution(solutions, reference)
  if (best) {
    return {
      joints: clampJointToLimits(best.jointsDeg),
      suggestion: null,
    }
  }

  const nearest = findNearestReachableCartesianPosition(pose, reference)
  if (nearest) {
    return {
      joints: null,
      suggestion: {
        which: 'goal',
        position: nearest.position,
        distanceMm: nearest.distanceMm,
      },
      message:
        `No valid IK solution at [${formatTriplet(pose.position)}]. ` +
        `Suggested nearest reachable XYZ: [${formatTriplet(nearest.position)}] ` +
        `(Δ≈${roundTo(nearest.distanceMm, 1)} mm, same orientation).`,
    }
  }

  return {
    joints: null,
    suggestion: null,
    message:
      `No valid IK solution at [${formatTriplet(pose.position)}]. ` +
      'Try changing orientation (rz, ry, rx) or move closer to the workspace center.',
  }
}

const defaultStartC: CartesianPose = jointToCartesianPose(defaultStartJ)
const defaultGoalC: CartesianPose = jointToCartesianPose(defaultGoalJ)

const homePreviewTrajectory: PlannedTrajectory = {
  kind: 'MoveJ',
  samples: [
    { t: 0, q: defaultStartJ },
    { t: 0.1, q: defaultStartJ },
  ],
}

function trajectoryHasOnlyFiniteValues(planned: PlannedTrajectory): boolean {
  return planned.samples.every((sample) => sample.q.every((value) => Number.isFinite(value)) && Number.isFinite(sample.t))
}

function clampJointToLimits(joints: JointVectorDeg): JointVectorDeg {
  const limits = robotParams().jointLimitsDeg
  const output = [...joints] as JointVectorDeg
  for (let i = 0; i < 6; i += 1) {
    output[i] = Math.max(limits.min[i], Math.min(limits.max[i], output[i]))
  }
  return output
}

type NumericInputProps = {
  value: number
  onValueChange: (value: number) => void
  min?: number
  max?: number
  step?: number
}

function NumericInput({ value, onValueChange, min, max, step }: NumericInputProps) {
  const [draft, setDraft] = useState(String(value))

  useEffect(() => {
    setDraft(String(value))
  }, [value])

  const onChange = (raw: string) => {
    setDraft(raw)
    const parsed = Number(raw)
    if (Number.isFinite(parsed)) {
      onValueChange(parsed)
    }
  }

  const onBlur = () => {
    const parsed = Number(draft)
    if (!Number.isFinite(parsed)) {
      setDraft(String(value))
    }
  }

  return (
    <input
      type="number"
      inputMode="decimal"
      value={draft}
      min={min}
      max={max}
      step={step}
      onChange={(e) => onChange(e.target.value)}
      onBlur={onBlur}
    />
  )
}

function App() {
  const [waypointMode, setWaypointMode] = useState<WaypointMode>('joint')
  const [motionType, setMotionType] = useState<'MoveJ' | 'MoveL'>('MoveJ')
  const [timeProfile, setTimeProfile] = useState<TimeProfile>('lspb')
  const [steps, setSteps] = useState(80)
  const [durationSec, setDurationSec] = useState(4)

  const [startJoint, setStartJoint] = useState<JointVectorDeg>(defaultStartJ)
  const [goalJoint, setGoalJoint] = useState<JointVectorDeg>(defaultGoalJ)
  const [startCartesian, setStartCartesian] = useState<CartesianPose>(defaultStartC)
  const [goalCartesian, setGoalCartesian] = useState<CartesianPose>(defaultGoalC)

  const [trajectory, setTrajectory] = useState<PlannedTrajectory | null>(homePreviewTrajectory)
  const [playing, setPlaying] = useState(false)
  const [message, setMessage] = useState('Ready')
  const [animationFrame, setAnimationFrame] = useState(`1/${homePreviewTrajectory.samples.length}`)
  const [validation, setValidation] = useState<ValidationSummary | null>(null)
  const [viewPreset, setViewPreset] = useState<'iso' | 'front' | 'side'>('iso')
  const [suggestedCartesian, setSuggestedCartesian] = useState<SuggestedCartesianPoint | null>(null)
  const cartesianBounds = useMemo(() => estimateCartesianWorkspaceBoundsMm(2500), [])

  const trajectoryFrames = useMemo(() => trajectory?.samples.map((sample) => sample.q) ?? [], [trajectory])
  const goalJointPose = useMemo(() => {
    const goalTransform = ABB_IRB120_FK(goalJoint)
    const goalPosition: [number, number, number] = [goalTransform[0][3], goalTransform[1][3], goalTransform[2][3]]
    const goalRpy = transformToRpyDegZYX(goalTransform)
    return { position: goalPosition, rpy: goalRpy }
  }, [goalJoint])

  const onViewerFrameChange = useCallback((index: number, count: number) => {
    setAnimationFrame(`${index + 1}/${count}`)
  }, [])

  const onWaypointModeChange = (nextMode: WaypointMode) => {
    setWaypointMode(nextMode)
    setPlaying(false)

    if (!trajectory || !trajectoryHasOnlyFiniteValues(trajectory) || trajectory.samples.length === 0) {
      setTrajectory(homePreviewTrajectory)
      setAnimationFrame(`1/${homePreviewTrajectory.samples.length}`)
      setMessage('Recovered to home preview after invalid trajectory state.')
    }
  }

  const onPlan = () => {
    try {
      let qStart: JointVectorDeg
      let qGoal: JointVectorDeg

      if (waypointMode === 'joint') {
        qStart = clampJointToLimits(startJoint)
        qGoal = clampJointToLimits(goalJoint)
        setSuggestedCartesian(null)
      } else {
        const startResolved = resolveJointFromPose(startCartesian)
        if (!startResolved.joints) {
          setSuggestedCartesian(startResolved.suggestion ? { ...startResolved.suggestion, which: 'start' } : null)
          throw new Error(startResolved.message ?? 'No valid IK solution for start Cartesian waypoint.')
        }
        qStart = startResolved.joints

        const goalResolved = resolveJointFromPose(goalCartesian, qStart)
        if (!goalResolved.joints) {
          setSuggestedCartesian(goalResolved.suggestion ? { ...goalResolved.suggestion, which: 'goal' } : null)
          throw new Error(goalResolved.message ?? 'No valid IK solution for goal Cartesian waypoint.')
        }
        qGoal = goalResolved.joints
        setSuggestedCartesian(null)
      }

      const planned =
        motionType === 'MoveJ'
          ? planMoveJ(qStart, qGoal, steps, durationSec, timeProfile)
          : planMoveL(qStart, qGoal, steps, durationSec, timeProfile)

      if (planned.samples.length === 0 || !trajectoryHasOnlyFiniteValues(planned)) {
        throw new Error('Planner produced invalid trajectory samples. Adjust waypoints and try again.')
      }

      setTrajectory(planned)
      setPlaying(false)
      setAnimationFrame(`1/${planned.samples.length}`)
      setMessage(`${planned.kind} planned with ${planned.samples.length} points using ${timeProfile} profile.`)
      setSuggestedCartesian(null)
    } catch (error) {
      setPlaying(false)
      if (!trajectory || !trajectoryHasOnlyFiniteValues(trajectory) || trajectory.samples.length === 0) {
        setTrajectory(homePreviewTrajectory)
        setAnimationFrame(`1/${homePreviewTrajectory.samples.length}`)
      }
      setMessage(error instanceof Error ? error.message : 'Planning failed.')
    }
  }

  const onApplySuggestedPoint = () => {
    if (!suggestedCartesian) {
      return
    }

    const roundedPosition: [number, number, number] = [
      roundTo(suggestedCartesian.position[0], 2),
      roundTo(suggestedCartesian.position[1], 2),
      roundTo(suggestedCartesian.position[2], 2),
    ]

    if (suggestedCartesian.which === 'start') {
      setStartCartesian((previous) => ({ ...previous, position: roundedPosition }))
    } else {
      setGoalCartesian((previous) => ({ ...previous, position: roundedPosition }))
    }

    setMessage(
      `Applied suggested ${suggestedCartesian.which} XYZ: [${formatTriplet(roundedPosition)}] ` +
      `(Δ≈${roundTo(suggestedCartesian.distanceMm, 1)} mm).`,
    )
    setSuggestedCartesian(null)
  }

  const runValidation = () => {
    const summary = runValidationSuite(250)
    setValidation(summary)
  }

  const onTogglePlay = () => {
    if (!trajectory) {
      setMessage('Plan Path first, then press Play Animation.')
      return
    }
    setPlaying((prev) => !prev)
  }

  const setJointValue = (
    which: 'start' | 'goal',
    index: number,
    nextValue: number,
  ) => {
    const setter = which === 'start' ? setStartJoint : setGoalJoint
    const source = which === 'start' ? startJoint : goalJoint
    const draft = [...source] as JointVectorDeg
    draft[index] = nextValue
    setter(draft)
  }

  const setCartesianValue = (
    which: 'start' | 'goal',
    group: 'position' | 'rpyZYX',
    index: number,
    nextValue: number,
  ) => {
    const source = which === 'start' ? startCartesian : goalCartesian
    const setter = which === 'start' ? setStartCartesian : setGoalCartesian
    const nextPose: CartesianPose = {
      position: [...source.position] as [number, number, number],
      rpyZYX: [...source.rpyZYX] as [number, number, number],
    }
    nextPose[group][index] = nextValue
    setter(nextPose)
  }

  const limits = robotParams().jointLimitsDeg
  const positionLabels = [
    `x (mm) [${cartesianBounds.x.min}, ${cartesianBounds.x.max}]`,
    `y (mm) [${cartesianBounds.y.min}, ${cartesianBounds.y.max}]`,
    `z (mm) [${cartesianBounds.z.min}, ${cartesianBounds.z.max}]`,
  ] as const
  const orientationLabels = ['rz (deg) [-180, 180]', 'ry (deg) [-180, 180]', 'rx (deg) [-180, 180]'] as const

  return (
    <main className="app">
      <h1>ABB IRB120 Motion Planning App</h1>
      <section className="card">
        <h2>Planner Controls</h2>
        <div className="row">
          <label>
            Waypoint type
            <select value={waypointMode} onChange={(e) => onWaypointModeChange(e.target.value as WaypointMode)}>
              <option value="joint">Joint angles</option>
              <option value="cartesian">Cartesian pose</option>
            </select>
          </label>
          <label>
            Motion
            <select value={motionType} onChange={(e) => setMotionType(e.target.value as 'MoveJ' | 'MoveL')}>
              <option value="MoveJ">MoveJ (joint space)</option>
              <option value="MoveL">MoveL (Cartesian space)</option>
            </select>
          </label>
          <label>
            Time profile
            <select value={timeProfile} onChange={(e) => setTimeProfile(e.target.value as TimeProfile)}>
              <option value="lspb">LSPB (parabolic blend)</option>
              <option value="cubic">Cubic polynomial</option>
            </select>
          </label>
          <label>
            Steps
            <NumericInput
              min={2}
              max={300}
              value={steps}
              step={1}
              onValueChange={(value) => setSteps(Math.max(2, Math.round(value)))}
            />
          </label>
          <label>
            Duration (s)
            <NumericInput
              min={1}
              max={30}
              value={durationSec}
              step={0.1}
              onValueChange={(value) => setDurationSec(Math.max(1, value))}
            />
          </label>
        </div>

        {waypointMode === 'joint' ? (
          <div className="grid2">
            <div>
              <h3>Start joint waypoint (deg)</h3>
              {startJoint.map((value, idx) => (
                <label className="row compact" key={`start-j-${idx}`}>
                  q{idx + 1} [{limits.min[idx]}, {limits.max[idx]}]
                  <NumericInput
                    value={value}
                    min={limits.min[idx]}
                    max={limits.max[idx]}
                    step={0.1}
                    onValueChange={(next) => setJointValue('start', idx, next)}
                  />
                </label>
              ))}
            </div>
            <div>
              <h3>Goal joint waypoint (deg)</h3>
              {goalJoint.map((value, idx) => (
                <label className="row compact" key={`goal-j-${idx}`}>
                  q{idx + 1} [{limits.min[idx]}, {limits.max[idx]}]
                  <NumericInput
                    value={value}
                    min={limits.min[idx]}
                    max={limits.max[idx]}
                    step={0.1}
                    onValueChange={(next) => setJointValue('goal', idx, next)}
                  />
                </label>
              ))}
              <p>
                Goal FK position (mm): [{formatTriplet(goalJointPose.position)}]
              </p>
              <p>
                Goal FK orientation rz,ry,rx (deg): [{formatTriplet(goalJointPose.rpy)}]
              </p>
            </div>
          </div>
        ) : (
          <div className="grid2">
            <div>
              <h3>Start Cartesian waypoint</h3>
              {positionLabels.map((name, idx) => (
                <label className="row compact" key={`start-p-${idx}`}>
                  {name}
                  <NumericInput
                    value={startCartesian.position[idx]}
                    step={0.1}
                    onValueChange={(next) => setCartesianValue('start', 'position', idx, next)}
                  />
                </label>
              ))}
              {orientationLabels.map((name, idx) => (
                <label className="row compact" key={`start-r-${idx}`}>
                  {name}
                  <NumericInput
                    value={startCartesian.rpyZYX[idx]}
                    step={0.1}
                    onValueChange={(next) => setCartesianValue('start', 'rpyZYX', idx, next)}
                  />
                </label>
              ))}
            </div>
            <div>
              <h3>Goal Cartesian waypoint</h3>
              {positionLabels.map((name, idx) => (
                <label className="row compact" key={`goal-p-${idx}`}>
                  {name}
                  <NumericInput
                    value={goalCartesian.position[idx]}
                    step={0.1}
                    onValueChange={(next) => setCartesianValue('goal', 'position', idx, next)}
                  />
                </label>
              ))}
              {orientationLabels.map((name, idx) => (
                <label className="row compact" key={`goal-r-${idx}`}>
                  {name}
                  <NumericInput
                    value={goalCartesian.rpyZYX[idx]}
                    step={0.1}
                    onValueChange={(next) => setCartesianValue('goal', 'rpyZYX', idx, next)}
                  />
                </label>
              ))}
            </div>
          </div>
        )}

        <div className="actions">
          <button onClick={onPlan}>Plan Path</button>
          <button onClick={onTogglePlay}>
            {playing ? 'Pause Animation' : 'Play Animation'}
          </button>
          {waypointMode === 'cartesian' && suggestedCartesian && (
            <button onClick={onApplySuggestedPoint}>
              Apply Suggested Point ({suggestedCartesian.which})
            </button>
          )}
          <span>Frame: {animationFrame}</span>
          <span>{message}</span>
        </div>
      </section>

      <section className="card">
        <h2>3D Robot Animation</h2>
        <p>Mouse: left drag = rotate, right drag = pan, wheel = zoom.</p>
        <div className="actions">
          <button onClick={() => setViewPreset('iso')}>Isometric View</button>
          <button onClick={() => setViewPreset('front')}>Front View</button>
          <button onClick={() => setViewPreset('side')}>Side View</button>
        </div>
        <RobotViewer
          trajectory={trajectoryFrames}
          playing={playing}
          viewPreset={viewPreset}
          onFrameChange={onViewerFrameChange}
          onPlaybackComplete={() => {
            setPlaying(false)
            setMessage('Animation complete. Press Play Animation to replay.')
          }}
        />
      </section>

      <section className="card chartCard">
        <h2>Joint-angle Plots</h2>
        <JointChart samples={trajectory?.samples ?? []} />
      </section>

      <section className="card">
        <h2>Validation (FK/IK)</h2>
        <div className="actions">
          <button onClick={runValidation}>Run Validation Suite</button>
          {validation && (
            <span className={validation.pass ? 'ok' : 'bad'}>
              {validation.pass ? 'PASS' : 'CHECK'} - Random success {validation.random.successRatePct.toFixed(1)}%
            </span>
          )}
        </div>

        {validation && (
          <>
            <p>
              Thresholds: pos ≤ {validation.thresholds.maxPosErrMm} mm, rot ≤ {validation.thresholds.maxRotErrFro},
              random success ≥ {validation.thresholds.minRandomSuccessPct}%.
            </p>
            <p>
              Random tests: {validation.random.solvedTargets}/{validation.random.sampleCount} targets solved
              with at least one valid IK solution.
            </p>
            <div className="grid2">
              {validation.deterministic.map((item) => (
                <div key={item.name} className="subCard">
                  <strong>{item.name}</strong>
                  <p>q: [{item.sourceQDeg.map((v) => v.toFixed(1)).join(', ')}]</p>
                  <p>IK candidates: {item.candidates}</p>
                  <p>Best pos err: {item.bestPosErrMm.toExponential(3)} mm</p>
                  <p>Best rot err: {item.bestRotErrFro.toExponential(3)}</p>
                </div>
              ))}
            </div>
          </>
        )}
      </section>
    </main>
  )
}

export default App
