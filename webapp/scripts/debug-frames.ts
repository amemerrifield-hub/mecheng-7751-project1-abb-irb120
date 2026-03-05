import { planMoveJ } from '../src/motion/planning'
const q0 = [0,-30,20,0,20,0] as [number,number,number,number,number,number]
const q1 = [45,10,-20,30,-25,40] as [number,number,number,number,number,number]
const tr = planMoveJ(q0,q1,100,6,'lspb')
console.log('samples', tr.samples.length)
console.log('first', tr.samples[0].q)
console.log('mid', tr.samples[Math.floor(tr.samples.length/2)].q)
console.log('last', tr.samples[tr.samples.length-1].q)
