import fs from 'node:fs';
import crypto from 'node:crypto';
import {modelsFrom,runScenario,presets} from './core.mjs';

const modelPath=new URL('../reports/sysid-model-03.json',import.meta.url);
const calibrationPath=new URL('../calibration.json',import.meta.url);
const modelBytes=fs.readFileSync(modelPath),gainBytes=fs.readFileSync(calibrationPath);
const models=modelsFrom(JSON.parse(modelBytes)),gains=JSON.parse(gainBytes).gains;
const results=[];
for(let motor=0;motor<3;motor++)for(const direction of [1,-1])for(const preset of ['load','mismatch','saturation','encoder']){
  const result=runScenario(models,gains,{...presets[preset],preset,motor,direction});
  results.push(result);
  console.log(`${preset} M${motor+1} ${direction}:`,Object.fromEntries(Object.entries(result.metrics).map(([k,v])=>[k,Math.round(v.load_window_mae_cps)])));
}
for(const preset of ['wall','slip','camera','lateral']){
  const result=runScenario(models,gains,{...presets[preset],preset});results.push(result);
  console.log(preset,result.metrics);
}
const hash=bytes=>crypto.createHash('sha256').update(bytes).digest('hex');
const output=new URL('../reports/simulation-results.json',import.meta.url);
const report={schema:'wbb-simulation-1',created_utc:new Date().toISOString(),
  model_sha256:hash(modelBytes),calibration_sha256:hash(gainBytes),
  interpretation:'Simulation only. Wall forces/inertia/adhesion/sensors are assumptions; no hardware commands or firmware changes.',
  scenarios:results.map(({options,seconds,metrics})=>({options,seconds,metrics}))};
fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
// A subset for a reproducible standalone plot, independent of browser rendering.
const chosen=results.filter(r=>(r.options.motor===0&&r.options.direction===1&&r.options.preset==='load')||['wall','slip','lateral'].includes(r.options.preset));
fs.writeFileSync(new URL('../reports/simulation-traces.json',import.meta.url),JSON.stringify(chosen));
