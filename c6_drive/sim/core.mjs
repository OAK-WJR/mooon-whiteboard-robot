// Pure deterministic simulation kernel: no DOM, network, serial, or hardware API.
export const DT = .01, MPC = .11938052 / 2800, R = .033738;
export const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
export const variants = ['legacy', 'model', 'eso'];
export const labels = {legacy:'Existing wheel-speed PI', model:'Model feedforward + PI', eso:'Model feedforward + PI + ESO'};
const sign = Math.sign;
const dot = (a,b) => a.reduce((s,x,i)=>s+x*b[i],0);
export function modelsFrom(data) {
  return [1,2,3].map(motor => [1,-1].map(direction => {
    const m=data.models.find(m=>m.motor===motor&&m.sign===direction);
    if(!m||!(m.gain_cps_per_pwm>0)||!(m.tau_s>0)||!Number.isFinite(m.bias_cps))
      throw Error('All six measured models are required');
    return {K:m.gain_cps_per_pwm,tau:m.tau_s,ks:Math.max(0,-m.bias_cps/m.gain_cps_per_pwm)};
  }));
}
export function wheelMap(theta) {
  const c=Math.cos(theta),s=Math.sin(theta);
  return [[-.8660254038,.5],[0,-1],[.8660254038,.5]].map(([x,y])=>[c*x-s*y,s*x+c*y,R]);
}
export const effectivePWM = (u,m) => sign(u)*Math.max(0,Math.abs(u)-m.ks);
// A single interpolation factor preserves the requested chassis acceleration.
export function rampVector(current,target,dt=DT,rate=12000){
  const delta=target.map((x,i)=>x-current[i]);
  const scale=Math.min(1,rate*dt/Math.max(1e-12,...delta.map(Math.abs)));
  return current.map((x,i)=>x+delta[i]*scale);
}

// Port of drive::Wheel::update(). Parity is checked against the actual C++ header.
export class LegacyWheel {
  constructor(gains) {this.gains=gains;this.reset();}
  reset(){this.ramp=this.integral=this.output=this.startupTime=0;this.direction=0;this.comp=0;}
  update(target,measured,dt=DT) {
    if(Math.abs(target)<1){this.reset();return 0;}
    const goal=target*this.ramp<0?0:target;
    this.ramp+=clamp(goal-this.ramp,-12000*dt,12000*dt);
    const dir=sign(this.ramp);
    if(dir&&dir!==this.direction){this.direction=dir;this.startupTime=.25;this.integral=0;}
    if(measured*dir>500)this.startupTime=0;
    const starting=this.startupTime>0&&dir;
    this.startupTime=Math.max(0,this.startupTime-dt);
    const [kp,ki,kv,ks]=this.gains[this.ramp<0?1:0];
    const e=this.ramp-measured,ff=kv*this.ramp+sign(this.ramp)*ks;
    const pre=ff+kp*e+this.integral;
    if(!starting&&!((pre>=255&&e>0)||(pre<=-255&&e<0)))
      this.integral=clamp(this.integral+ki*e*dt,-100,100);
    let desired=clamp(ff+kp*e+this.integral,-255,255);
    desired=this.ramp>=0?Math.max(0,desired):Math.min(0,desired);
    if(starting)desired=dir>0?Math.max(140,desired):Math.min(-140,desired);
    this.output+=clamp(desired-this.output,-1200*dt,1200*dt);
    return this.output;
  }
}

// Discrete augmented observer: v+ = a*v + b*(u_eff - q), q+ = q.
// Measurement correction poles are both exp(-bandwidth*dt). q is PWM-equivalent
// disturbance, not motor torque. The observer receives LAST APPLIED integer PWM.
export class Observer {
  constructor(){this.reset();}
  reset(){this.v=0;this.q=0;this.initialized=false;}
  update(measured,applied,m,dt,learn=true) {
    if(!this.initialized){this.v=measured;this.initialized=true;return 0;}
    const a=Math.exp(-dt/m.tau),b=m.K*(1-a),p=Math.exp(-12*dt);
    const predicted=a*this.v+b*(effectivePWM(applied,m)-this.q);
    const e=measured-predicted,l1=1-p*p/a,l2=-((1-p)**2)/b;
    this.v=predicted+l1*e;
    if(learn)this.q=clamp(this.q+l2*e,-90,90);
    return this.q;
  }
}

export class ModelWheel {
  constructor(pair,useESO){this.pair=pair;this.useESO=useESO;this.observer=new Observer();this.reset();}
  reset(){this.output=this.integral=this.ramp=this.comp=0;this.observer.reset();}
  update(target,measured,dt=DT,{hold=false,learn=true}={}) {
    if(!Number.isFinite(measured)){this.reset();return 0;}
    if(Math.abs(target)<1&&!hold){this.reset();return 0;}
    const m=this.pair[(this.output||this.ramp||target||measured)<0?1:0];
    const load=this.useESO?this.observer.update(measured,Math.round(this.output),m,dt,learn):0;
    this.comp+=clamp(load-this.comp,-180*dt,180*dt);
    this.ramp+=clamp(target-this.ramp,-12000*dt,12000*dt);
    const ref=this.pair[this.ramp<0?1:0],lambda=.30;
    // Same PI gains in model and ESO variants: isolate observer contribution.
    const kp=ref.tau/(ref.K*lambda),ki=1/(ref.K*lambda),e=this.ramp-measured;
    const linear=this.ramp/ref.K+kp*e+this.integral+this.comp;
    const desired=linear+sign(linear)*ref.ks;
    if(learn&&!((desired>255&&e>0)||(desired< -255&&e<0)))
      this.integral=clamp(this.integral+ki*e*dt,-100,100);
    const next=this.ramp/ref.K+kp*e+this.integral+this.comp;
    const bounded=clamp(next+sign(next)*ref.ks,-255,255);
    this.output+=clamp(bounded-this.output,-1200*dt,1200*dt);
    return this.output;
  }
}

export const defaults={mode:'bench',preset:'load',motor:0,direction:1,speed:.18,
  load:35,mass:.30,normal:12,mu:.65,force:4,voltage:1,gainScale:1,tauScale:1,
  noise:25,camera:true,hold:true,sync:true,cameraDelay:.08,seed:42};
export const presets={
  load:{mode:'bench',load:35},
  mismatch:{mode:'bench',load:25,gainScale:.75,tauScale:1.4},
  saturation:{mode:'bench',load:145,voltage:.75},
  encoder:{mode:'bench',load:35},
  wall:{mode:'wall',mass:.30,normal:12,mu:.65,force:4},
  slip:{mode:'wall',mass:.30,normal:12,mu:.65,force:4},
  camera:{mode:'wall',mass:.30,normal:12,mu:.65,force:4},
  lateral:{mode:'wall',mass:.30,normal:12,mu:.65,force:4,gravity:0,camera:false,hold:false},
  manual:{mode:'wall',mass:.30,normal:12,mu:.65,force:4}
};
function random(seed) {
  let a=seed>>>0;
  return ()=>{a=(Math.imul(1664525,a)+1013904223)>>>0;return a/4294967296;};
}
export function environment(options,time) {
  const o={...options};
  o.loadNow=time>=3&&time<7?o.load:0;
  if(o.preset==='slip'&&time>=4)o.mu*=.12;
  o.encoderOK=!(o.preset==='encoder'&&time>=5);
  o.cameraOK=o.camera&&!(o.preset==='camera'&&time>=4);
  return o;
}

// Approximate three-wheel planar rigid body with constrained friction impulses.
// K/tau/ks are measured; force coefficient, reflected inertia, coast drag, mass,
// normal-force distribution and Coulomb contact law are ASSUMPTIONS, not identified.
export class Plant {
  constructor(models,options) {
    this.models=models;this.options=options;this.w=[0,0,0]; // tangential wheel m/s
    this.pose=[0,0,Math.PI/2];this.velocity=[0,0,0];this.force=[0,0,0];
    this.slip=[0,0,0];this.normal=[0,0,0];
  }
  step(pwm,env,dt=DT) {
    if(env.mode==='bench'){
      for(let i=0;i<3;i++){
        const m=this.models[i][(pwm[i]||this.w[i])<0?1:0];
        const dir=env.direction;
        const input=effectivePWM(pwm[i]*env.voltage,m)-(i===env.motor?dir*env.loadNow:0);
        const a=Math.exp(-dt/(m.tau*env.tauScale));
        this.w[i]=a*this.w[i]+(1-a)*m.K*env.gainScale*MPC*input;
      }
      return;
    }
    // 1ms contact steps; impulses solved with projected Gauss-Seidel.
    const n=Math.ceil(dt/.001),h=dt/n;
    for(let k=0;k<n;k++)this.contactStep(pwm,env,h);
  }
  contactStep(pwm,env,h) {
    const A=wheelMap(this.pose[2]),invBody=[1/env.mass,1/env.mass,1/(env.mass*.055**2)];
    const invWheel=[];
    for(let i=0;i<3;i++){
      const m=this.models[i][(pwm[i]||this.w[i])<0?1:0];
      // Chosen maximum tangential force maps effective duty to newtons.
      const cu=env.force/(255-m.ks),b=cu/(m.K*MPC*env.gainScale);
      const mass=b*m.tau*env.tauScale;invWheel[i]=1/mass;
      const drag=Math.abs(pwm[i])<.5?.15:1;
      const drive=cu*effectivePWM(pwm[i]*env.voltage,m)-drag*b*this.w[i];
      this.w[i]+=h*drive/mass;
    }
    this.velocity[1]-=(env.gravity??9.81)*h;
    // Gravity/body state are NEVER exposed to the wheel observer/controller.
    const D=A.map((ai,i)=>A.map((aj,j)=>dot(ai.map((x,k)=>x*invBody[k]),aj)+(i===j?invWheel[i]:0)));
    const residual=A.map((ai,i)=>this.w[i]-dot(ai,this.velocity));
    const cap=env.mu*env.normal/3*h,impulses=[0,0,0];
    for(let sweep=0;sweep<12;sweep++)for(let i=0;i<3;i++)
      impulses[i]=clamp(impulses[i]+(residual[i]-dot(D[i],impulses))/D[i][i],-cap,cap);
    for(let i=0;i<3;i++){
      this.w[i]-=impulses[i]*invWheel[i];
      for(let k=0;k<3;k++)this.velocity[k]+=invBody[k]*A[i][k]*impulses[i];
      this.force[i]=impulses[i]/h;this.normal[i]=env.normal/3;
    }
    for(let k=0;k<3;k++)this.pose[k]+=h*this.velocity[k];
    this.slip=A.map((ai,i)=>this.w[i]-dot(ai,this.velocity));
  }
}

class Camera {
  constructor(){this.queue=[];this.last=null;this.next=0;}
  sample(plant,t,env,rng){
    if(env.cameraOK&&t>=this.next){
      this.next=t+.05;
      this.queue.push({capture:t,available:t+env.cameraDelay,
        pose:plant.pose.map((x,i)=>x+(rng()-.5)*(i===2?.002:.0004))});
    }
    // Velocity is a camera-like finite difference, never truth velocity.
    while(this.queue.length&&this.queue[0].available<=t){
      const sample=this.queue.shift();
      sample.velocity=this.last?sample.pose.map((p,i)=>(p-this.last.pose[i])/(sample.capture-this.last.capture)):[0,0,0];
      this.last=sample;
    }
    return this.last&&t-this.last.capture<.30?this.last:null;
  }
}

export class Simulation {
  constructor(models,gains,variant,options={}) {
    if(!variants.includes(variant))throw Error('Unknown controller');
    this.variant=variant;this.models=models;this.options={...defaults,...options};
    this.plant=new Plant(models,this.options);this.rng=random(this.options.seed);
    this.controllers=models.map((pair,i)=>variant==='legacy'?new LegacyWheel(gains[i]):new ModelWheel(pair,variant==='eso'));
    this.camera=new Camera();this.t=0;this.goal=[0,0,Math.PI/2];this.fault=null;
    this.sensorHeading=Math.PI/2;this.pwm=[0,0,0];this.last=null;this.stopped=false;
    this.sharedTargets=[0,0,0];
    this.slipDetected=false;this.slipTimer=0;
  }
  stop(){this.stopped=true;this.controllers.forEach(c=>c.reset());this.pwm=[0,0,0];this.sharedTargets=[0,0,0];}
  step(manual=null) {
    if(this.ended){this.t+=DT;return this.last={...this.last,t:this.t};}
    const env=environment(this.options,this.t),measurement=this.plant.w.map(v=>v/MPC+(this.rng()-.5)*2*env.noise);
    const cam=this.camera.sample(this.plant,this.t,env,this.rng);
    let targets=[0,0,0],hold=false;
    if(env.mode==='bench')targets[env.motor]=env.direction*5200;
    else {
      // Commands are in board coordinates (W up), not robot coordinates.
      const automatic=env.preset==='lateral'?
        [this.t>=1&&this.t<3?env.speed:this.t>=4&&this.t<6?-env.speed:0,0,0]:
        (this.t>=1&&this.t<3?[0,env.speed,0]:[0,0,0]);
      const command=manual??automatic;
      for(let k=0;k<3;k++)this.goal[k]+=command[k]*DT;
      if(cam&&this.variant!=='legacy')this.sensorHeading=cam.pose[2];
      const A=wheelMap(this.sensorHeading);
      if(cam&&this.variant!=='legacy'){
        const discrepancy=A.map((a,i)=>measurement[i]*MPC-dot(a,cam.velocity));
        this.slipTimer=discrepancy.some(x=>Math.abs(x)>.08)?this.slipTimer+DT:0;
        this.slipDetected=this.slipTimer>.2;
      }else{this.slipDetected=false;this.slipTimer=0;}
      hold=env.hold&&this.variant!=='legacy'&&!!cam;
      let requested=[...command];
      if(hold)requested=requested.map((v,i)=>clamp(v+2*(this.goal[i]-cam.pose[i]),i===2?-1.2:-.30,i===2?1.2:.30));
      if(this.slipDetected)requested=requested.map(v=>v*.35);
      // Candidate HOLD depends on a new, explicitly simulated position sensor.
      if(env.hold&&this.options.camera&&this.variant!=='legacy'&&!cam&&this.t>.35)this.fault='POSITION_FEEDBACK_LOST';
      targets=A.map(a=>dot(a,requested)/MPC);
      const scale=Math.min(1,10000/Math.max(1,...targets.map(Math.abs)));
      targets=targets.map(x=>x*scale);
    }
    if(!env.encoderOK)this.fault='ENCODER_INVALID';
    const requestedTargets=[...targets];
    if(this.variant!=='legacy'&&env.sync){
      this.sharedTargets=rampVector(this.sharedTargets,targets);
      targets=[...this.sharedTargets];
    }
    const halted=this.stopped||this.fault;
    if(halted){this.pwm=[0,0,0];this.controllers.forEach(c=>c.reset());}
    else this.pwm=this.controllers.map((c,i)=>Math.round(c.update(targets[i],measurement[i],DT,{hold,learn:!this.slipDetected})));
    this.plant.step(this.pwm,env,DT);
    this.t+=DT;
    if(env.mode==='wall'&&this.plant.pose[1]<-.60){
      this.ended=true;this.stop();
    }
    const maxPWM=Math.max(...this.pwm.map(Math.abs));
    this.last={t:this.t,variant:this.variant,targets,requestedTargets,speed:this.plant.w.map(v=>v/MPC),pwm:[...this.pwm],
      compensation:this.controllers.map(c=>c.comp),integral:this.controllers.map(c=>c.integral),
      pose:[...this.plant.pose],velocity:[...this.plant.velocity],goal:[...this.goal],
      slip:[...this.plant.slip],slipDetected:this.slipDetected,cameraFresh:!!cam,
      friction:env.mu,load:env.loadNow,fault:this.fault,stopped:this.stopped,ended:!!this.ended,
      status:this.ended?'FELL_OFF_BOARD':this.fault||(this.stopped?'OUTPUT_OFF':this.slipDetected?'SLIP_DETECTED':maxPWM>=254?'SATURATED':hold?'SIMULATED_HOLD':'RUNNING')};
    if([...this.last.speed,...this.last.pose,...this.last.pwm].some(x=>!Number.isFinite(x)))throw Error('Nonfinite simulation state');
    return this.last;
  }
}

export function runScenario(models,gains,options={},seconds=10) {
  const result={options:{...defaults,...options},seconds,traces:{},metrics:{}};
  for(const variant of variants){
    const sim=new Simulation(models,gains,variant,options),trace=[];
    for(let i=0;i<Math.round(seconds/DT);i++)trace.push(sim.step());
    result.traces[variant]=trace;
    const motor=sim.options.motor,window=trace.filter(f=>f.t>=3&&f.t<5);
    const error=window.map(f=>Math.abs(f.speed[motor]-f.targets[motor]));
    result.metrics[variant]={load_window_mae_cps:error.reduce((a,b)=>a+b,0)/error.length,
      peak_pwm:Math.max(...trace.flatMap(f=>f.pwm.map(Math.abs))),
      saturation_seconds:trace.filter(f=>f.pwm.some(u=>Math.abs(u)>=254)).length*DT,
      final_position_error_m:Math.hypot(trace.at(-1).pose[0]-trace.at(-1).goal[0],trace.at(-1).pose[1]-trace.at(-1).goal[1]),
      final_y_m:trace.at(-1).pose[1],slip_detected:trace.some(f=>f.slipDetected),
      fell_off_board:trace.some(f=>f.ended),
      peak_cross_track_m:Math.max(...trace.map(f=>Math.abs(f.pose[1]-f.goal[1]))),
      peak_heading_error_deg:Math.max(...trace.map(f=>Math.abs(f.pose[2]-f.goal[2])*180/Math.PI)),
      fault:trace.at(-1).fault};
  }
  return result;
}
