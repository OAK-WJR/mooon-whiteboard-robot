// Reuse the tested browser wheel controller inside the independent MuJoCo plant.
// JSON lines over a local subprocess pipe; this process has no hardware transport.
import readline from 'node:readline';
import fs from 'node:fs';
import {modelsFrom,ModelWheel,rampVector} from '../core.mjs';
const models=modelsFrom(JSON.parse(fs.readFileSync(new URL('../../reports/sysid-model-03.json',import.meta.url))));
let wheels=[],targets=[0,0,0];
for await(const line of readline.createInterface({input:process.stdin})){
  try{
    const a=JSON.parse(line);
    if(a.reset){wheels=models.map(pair=>new ModelWheel(pair,a.variant==='eso'));targets=[0,0,0];console.log('{}');continue;}
    if(a.stop){wheels.forEach(w=>w.reset());targets=[0,0,0];console.log(JSON.stringify({pwm:[0,0,0],comp:[0,0,0],targets}));continue;}
    targets=rampVector(targets,a.targets,.01,a.rate??12000);
    const pwm=wheels.map((w,i)=>Math.round(w.update(targets[i],a.measured[i],.01,{hold:a.hold,learn:a.learn})));
    console.log(JSON.stringify({pwm,comp:wheels.map(w=>w.comp),targets}));
  }catch(e){console.log(JSON.stringify({error:String(e)}));}
}
