import { runValidationSuite } from '../src/kinematics/validation'

const summary = runValidationSuite(300)

console.log('=== ABB IRB120 FK/IK VALIDATION ===')
console.log(
  `Thresholds: pos <= ${summary.thresholds.maxPosErrMm} mm | rot <= ${summary.thresholds.maxRotErrFro} | random >= ${summary.thresholds.minRandomSuccessPct}%`,
)

console.log('\nDeterministic cases:')
for (const item of summary.deterministic) {
  console.log(
    `- ${item.name}: candidates=${item.candidates}, posErr=${item.bestPosErrMm.toExponential(3)} mm, rotErr=${item.bestRotErrFro.toExponential(3)}`,
  )
}

console.log('\nRandom test summary:')
console.log(
  `- solved targets=${summary.random.solvedTargets}/${summary.random.sampleCount}, success=${summary.random.successRatePct.toFixed(2)}%`,
)

console.log(`\nRESULT: ${summary.pass ? 'PASS' : 'CHECK RESULTS'}`)
