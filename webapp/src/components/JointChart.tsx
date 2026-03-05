import {
  CategoryScale,
  Chart as ChartJS,
  Legend,
  LinearScale,
  LineElement,
  PointElement,
  Tooltip,
} from 'chart.js'
import { Line } from 'react-chartjs-2'
import type { TimedJointState } from '../motion/planning'

ChartJS.register(CategoryScale, LinearScale, PointElement, LineElement, Tooltip, Legend)

interface JointChartProps {
  samples: TimedJointState[]
}

const colors = ['#ef4444', '#f59e0b', '#10b981', '#0ea5e9', '#6366f1', '#d946ef']

export function JointChart({ samples }: JointChartProps) {
  if (samples.length === 0) {
    return <p>No trajectory yet.</p>
  }

  const labels = samples.map((sample) => sample.t.toFixed(2))
  const datasets = [0, 1, 2, 3, 4, 5].map((jointIdx) => ({
    label: `q${jointIdx + 1}`,
    data: samples.map((sample) => sample.q[jointIdx]),
    borderColor: colors[jointIdx],
    backgroundColor: colors[jointIdx],
    borderWidth: 2,
    pointRadius: 0,
    tension: 0.15,
  }))

  return (
    <Line
      data={{ labels, datasets }}
      options={{
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: { position: 'bottom' },
        },
        scales: {
          x: { title: { display: true, text: 'Time (s)' } },
          y: { title: { display: true, text: 'Joint angle (deg)' } },
        },
      }}
    />
  )
}
