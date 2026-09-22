import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {execFileSync} from 'node:child_process';
import {DT,MPC,modelsFrom,LegacyWheel,Observer,Plant,Simulation,runScenario,defaults,presets,variants,rampVector,wheelMap} from '../sim/core.mjs';
const models=modelsFrom(JSON.parse(fs.readFileSync(new URL('../reports/sysid-model-03.json',import.meta.url))));
const gains=JSON.parse(fs.readFileSync(new URL('../calibration.json',import.meta.url))).gains;
const root=path.resolve(new URL('../../',import.meta.url).pathname);
const tmp=fs.mkdtempSync(path.join(os.tmpdir(),'wbb-sim-test-'));
try {
  execFileSync('c++',['-std=c++17','-fsanitize=address,undefined','-I',path.join(root,'c6_drive/include'),
    path.join(root,'c6_drive/tests/sim_baseline.cpp'),'-o',path.join(tmp,'baseline')]);
  const controllers=gains.map(g=>new LegacyWheel(g)),expected=[],lines=[];
  for(let n=0;n<1300;n++)for(let i=0;i<3;i++){
    const target=n%250>210?0:(Math.floor(n/120)%2?-6000:6000);
    const measured=Math.sin(n*.037+i)*7000,dt=.01;
    controllers[i].update(target,measured,dt);
    expected.push([controllers[i].output,controllers[i].ramp,controllers[i].integral]);
    lines.push(`${i} ${target} ${measured} ${dt}`);
  }
  const actual=execFileSync(path.join(tmp,'baseline'),{input:gains.flat(2).join(' ')+'\n'+lines.join('\n')+'\n',encoding:'utf8'}).trim().split('\n').map(x=>x.split(' ').map(Number));
  assert.equal(actual.length,expected.length);
  actual.forEach((row,n)=>row.forEach((x,i)=>assert(Math.abs(x-expected[n][i])<.06,`C++ parity sample ${n}, field ${i}`)));
  console.log('PASS: 3,900 steps match the actual firmware Wheel controller');
} finally {fs.rmSync(tmp,{recursive:true,force:true});}

// An analytic free-wheel response validates units and exact integration separately.
{
  const plant=new Plant(models,defaults),m=models[0][0],u=180;
  for(let i=0;i<100;i++)plant.step([u,0,0],{...defaults,loadNow:0},DT);
  const expected=m.K*MPC*(u-m.ks)*(1-Math.exp(-1/m.tau));
  assert(Math.abs(plant.w[0]-expected)<1e-10);
}
// Contact cannot create friction without normal force; frictionless gravity is analytic.
{
  const plant=new Plant(models,defaults),env={...defaults,mode:'wall',mu:0,normal:0};
  for(let i=0;i<100;i++)plant.step([200,0,-200],env,DT);
  assert(Math.abs(plant.velocity[1]+9.81)<1e-8);
  assert(plant.force.every(x=>x===0));
  const p=new Plant(models,defaults),e={...defaults,mode:'wall'};
  for(let i=0;i<300;i++){
    p.step([200,-180,150],e,DT);
    assert(p.force.every(f=>Math.abs(f)<=e.mu*e.normal/3+1e-9));
  }
}
// Observer cannot access injected load; only sensor output and actual input are passed.
{
  const m=models[0][0],observer=new Observer();let v=5000;
  for(let i=0;i<500;i++){
    const a=Math.exp(-DT/m.tau);v=a*v+(1-a)*m.K*(180-m.ks-30);
    observer.update(v,180,m,DT);
  }
  assert(Math.abs(observer.q-30)<.1);
  const q=observer.q;observer.update(0,255,m,DT,false);assert.equal(observer.q,q);
}

let cases=0;
for(let motor=0;motor<3;motor++)for(const direction of [1,-1])for(const preset of ['load','mismatch','saturation','encoder']){
  const r=runScenario(models,gains,{...presets[preset],preset,motor,direction});cases++;
  for(const v of variants){
    assert(r.metrics[v].peak_pwm<=255);
    assert(r.traces[v].every(f=>f.compensation.every(x=>Math.abs(x)<=90+1e-9)));
  }
  if(['load','mismatch'].includes(preset))
    assert(r.metrics.eso.load_window_mae_cps<.65*r.metrics.model.load_window_mae_cps);
  if(preset==='saturation'){
    assert(r.metrics.eso.load_window_mae_cps>1500,'Cannot claim tracking under insufficient actuation');
    assert(r.metrics.eso.saturation_seconds>1);
  }
  if(preset==='encoder')for(const v of variants)
    assert(r.traces[v].filter(f=>f.t>5.02).every(f=>f.pwm.every(x=>x===0)));
}
for(const preset of ['wall','slip','camera','lateral']){
  const r=runScenario(models,gains,{...presets[preset],preset});cases++;
  if(preset==='wall'){
    assert(r.metrics.legacy.fell_off_board);
    assert(!r.metrics.model.fell_off_board&&!r.metrics.eso.fell_off_board);
  }
  if(preset==='slip'){
    assert(r.metrics.eso.slip_detected);
    assert(r.metrics.eso.fell_off_board,'Insufficient traction must still allow falling');
  }
  if(preset==='camera')for(const v of ['model','eso']){
    assert.equal(r.metrics[v].fault,'POSITION_FEEDBACK_LOST');
    assert(r.traces[v].filter(f=>f.t>4.4).every(f=>f.pwm.every(x=>x===0)));
  }
  if(preset==='lateral'){
    const independent=runScenario(models,gains,{...presets.lateral,preset:'lateral',sync:false});
    for(const v of ['model','eso'])assert(r.metrics[v].peak_cross_track_m<.3*independent.metrics[v].peak_cross_track_m);
  }
}
// A pure translation must not acquire commanded yaw during the shared ramp.
{
  const desired=wheelMap(Math.PI/2).map(a=>a[0]*.18/MPC);
  const ramp=rampVector([0,0,0],desired);
  assert(Math.abs(ramp.reduce((a,b)=>a+b,0))<1e-8);
  assert(Math.abs(ramp[0]/desired[0]-ramp[1]/desired[1])<1e-12);
  const old=desired.map(x=>Math.max(-120,Math.min(120,x)));
  assert(Math.abs(old.reduce((a,b)=>a+b,0))>100,'Independent wheel ramps break the translation ratio');
}
// Stop stays latched, and absent external feedback must not be replaced with truth.
{
  const sim=new Simulation(models,gains,'eso',{...presets.manual,preset:'manual',camera:false});
  for(let i=0;i<100;i++)sim.step([0,.18,0]);
  assert(!sim.last.cameraFresh&&!sim.slipDetected);
  sim.stop();for(let i=0;i<50;i++)assert(sim.step([0,.18,0]).pwm.every(u=>u===0));
}
console.log(`PASS: ${cases} scenarios; limits, model mismatch, sensor loss, slip and falling`);
