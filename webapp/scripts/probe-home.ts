import { ABB_IRB120_FK } from '../src/kinematics/abb120'
const qs: [number,number,number,number,number,number][] = [
  [0,0,0,0,0,0],
  [10,0,0,0,0,0],
  [0,10,0,0,0,0],
  [0,0,10,0,0,0],
  [0,0,0,10,0,0],
  [0,0,0,0,10,0],
  [0,0,0,0,0,10],
]
for (const q of qs){
  const T = ABB_IRB120_FK(q)
  const p = [T[0][3],T[1][3],T[2][3]].map(v=>Number(v.toFixed(3)))
  console.log(q.join(','), '=>', p.join(','))
}
