import {modelsFrom,runScenario,presets,variants,wheelMap,R} from './core.mjs';
const [modelData,calibration]=await Promise.all([fetch('../reports/sysid-model-03.json').then(r=>r.json()),fetch('../calibration.json').then(r=>r.json())]);
const models=modelsFrom(modelData),results={};
for(const scene of ['load','lateral','slip'])results[scene]=runScenario(models,calibration.gains,{...presets[scene],preset:scene});
const canvas=document.getElementById('film'),c=canvas.getContext('2d');
const color={legacy:'#e1a47b',model:'#7faeee',eso:'#72debd'},names={legacy:'Original wheel PI',model:'Model feedforward + PI',eso:'Model PI + ESO'};
let scene='load',time=0,playing=false,before=0;
function text(s,x,y,size=24,fill='#dce5ef',weight=400){c.fillStyle=fill;c.font=`${weight} ${size}px -apple-system, BlinkMacSystemFont, sans-serif`;c.textAlign='left';c.fillText(s,x,y);}
function panel(x,y,w,h){c.fillStyle='#121e2a';c.beginPath();c.roundRect(x,y,w,h,16);c.fill();c.strokeStyle='#26394b';c.lineWidth=1;c.stroke();}
function lines(strings,x,y,size=23,fill='#91a7bc',gap=34){strings.forEach((s,i)=>text(s,x,y+i*gap,size,fill));}
function chart(x,y,w,h,curves,lo,hi,title,target=null){
  text(title,x,y-18,22,'#a5b8ca',500);const left=x+60,right=x+w-18,top=y+8,bottom=y+h-33;
  const X=t=>left+t/10*(right-left),Y=v=>bottom-(v-lo)/(hi-lo)*(bottom-top);
  for(let i=0;i<=4;i++){
    const value=lo+(hi-lo)*i/4;c.strokeStyle='#283a4a';c.lineWidth=1;c.beginPath();c.moveTo(left,Y(value));c.lineTo(right,Y(value));c.stroke();
    text(Math.abs(value)>=100?value.toFixed(0):value.toFixed(2),x,Y(value)+6,17,'#7690a7');
  }
  for(let i=0;i<=5;i++)text(String(i*2),X(i*2)-5,y+h-5,17,'#7690a7');
  if(target!==null){c.setLineDash([6,7]);c.strokeStyle='#6f859a';c.beginPath();c.moveTo(left,Y(target));c.lineTo(right,Y(target));c.stroke();c.setLineDash([]);}
  c.save();c.beginPath();c.rect(left,top,right-left,bottom-top);c.clip();
  curves.forEach(curve=>{c.strokeStyle=curve.color;c.lineWidth=3;c.setLineDash(curve.dash??[]);c.beginPath();
    curve.data.filter(f=>f[0]<=time).forEach(([t,v],i)=>{if(i)c.lineTo(X(t),Y(v));else c.moveTo(X(t),Y(v));});c.stroke();});
  c.restore();c.setLineDash([]);
}
function board(result,lateral){
  const x=88,y=287,w=1110,h=440;
  const range=lateral?[-.22,.57,-.19,.14]:[-.32,.32,-.65,.50];
  const scale=Math.min(w/(range[1]-range[0]),h/(range[3]-range[2]));
  const X=a=>x+(w-(range[1]-range[0])*scale)/2+(a-range[0])*scale;
  const Y=a=>y+(h-(range[3]-range[2])*scale)/2+(range[3]-a)*scale;
  const spacing=lateral?.05:.1;
  c.save();c.beginPath();c.rect(x,y,w,h);c.clip();
  c.strokeStyle='#283b4b';c.lineWidth=1;
  for(let a=Math.ceil(range[0]/spacing)*spacing;a<range[1];a+=spacing){c.beginPath();c.moveTo(X(a),Y(range[2]));c.lineTo(X(a),Y(range[3]));c.stroke();}
  for(let a=Math.ceil(range[2]/spacing)*spacing;a<range[3];a+=spacing){c.beginPath();c.moveTo(X(range[0]),Y(a));c.lineTo(X(range[1]),Y(a));c.stroke();}
  const idx=Math.min(999,Math.max(0,Math.floor(time*100)-1)),selected=lateral?variants:['eso'];
  for(const v of selected){
    const history=result.traces[v].slice(0,idx+1);c.strokeStyle=color[v];c.lineWidth=3;c.beginPath();history.forEach((f,i)=>{if(i)c.lineTo(X(f.pose[0]),Y(f.pose[1]));else c.moveTo(X(f.pose[0]),Y(f.pose[1]));});c.stroke();
    const f=history.at(-1),[px,py,theta]=f.pose;
    c.save();c.translate(X(px),Y(py));c.rotate(-theta);c.strokeStyle=color[v];c.fillStyle=color[v]+'25';c.lineWidth=3;
    c.beginPath();c.moveTo(28,0);c.lineTo(-18,23);c.lineTo(-18,-23);c.closePath();c.fill();c.stroke();
    for(const a of wheelMap(0)){const xx=R*a[1]*scale*.6,yy=R*a[0]*scale*.6;c.fillStyle=color[v];c.fillRect(xx-4,yy-4,8,8);}c.restore();
    text(v==='legacy'?'PI':v==='model'?'MODEL':'ESO',X(px)+35,Y(py)+(v==='legacy'?28:v==='model'?2:-24),19,color[v],600);
  }
  const goal=result.traces.eso[idx].goal;c.strokeStyle='#dce5ef';c.setLineDash([4,6]);c.beginPath();c.arc(X(goal[0]),Y(goal[1]),12,0,Math.PI*2);c.stroke();c.setLineDash([]);
  c.restore();
  text(lateral?'TOP VIEW  /  5 cm grid  /  position feedback OFF':'VERTICAL SURFACE  /  10 cm grid  /  simulated position feedback',100,265,20,'#90a7bb');
  if(!lateral)text('Gravity ↓',985,320,24,'#e1a47b');
}
function render(){
  const result=results[scene];c.fillStyle='#0b1118';c.fillRect(0,0,1920,1080);
  text('WHITEBOARD BOT  /  CONTROL STUDY',64,58,20,'#72debd',600);
  const titles={load:'Compensating for an unknown load',lateral:'Keeping three wheels coordinated',slip:'Testing the limits of compensation'};
  text(titles[scene],64,126,52,'#edf2f7',600);
  const subtitles={load:'Measured motor model · identical PI gains · bounded disturbance observer',lateral:'Right–left maneuver · independent wheel dynamics · shared acceleration ramp',slip:'A friction drop can defeat a controller that tracks wheel speed correctly'};
  text(subtitles[scene],67,171,24,'#8fa6bd');
  c.fillStyle='#193229';c.beginPath();c.roundRect(1430,34,420,40,20);c.fill();text('SIMULATION  /  NO HARDWARE CONNECTION',1451,60,17,'#8bdfc5',500);
  panel(64,215,1170,726);panel(1260,215,596,726);
  const curves=field=>variants.map(v=>({color:color[v],data:result.traces[v].map(f=>[f.t,field(f)])}));
  if(scene==='load'){
    let lx=95;for(const v of variants){c.fillStyle=color[v];c.fillRect(lx,246,10,10);text(names[v],lx+20,257,19,'#abc0d0');lx+=v==='legacy'?260:350;}
    chart(92,310,1105,280,curves(f=>f.speed[0]),0,6500,'Wheel speed · encoder counts/s',5200);
    chart(92,657,525,232,curves(f=>f.pwm[0]),0,255,'Applied PWM');
    chart(667,657,530,232,[{color:color.eso,data:result.traces.eso.map(f=>[f.t,f.compensation[0]])},
      {color:'#8494a3',dash:[6,5],data:result.traces.eso.map(f=>[f.t,f.load])}],-5,45,'Estimated load · PWM equivalent');
    text('LOAD DISTURBANCE',1300,267,20,'#72debd',600);
    text('Motor 1 / positive direction',1300,312,28,'#e5edf5',500);
    lines(['3 s: apply +35 PWM-equivalent load','7 s: remove the load'],1300,366,24);
    text('2 s post-load mean absolute error',1300,492,23,'#91a7bc');
    text(`${result.metrics.model.load_window_mae_cps.toFixed(0)} → ${result.metrics.eso.load_window_mae_cps.toFixed(0)}`,1300,563,62,'#72debd',500);
    text('counts/s  ·  model PI → model PI + ESO',1300,605,21,'#a7b9c9');
    lines(['The observer estimates the extra load','from wheel speed and applied PWM.','It does not receive the injected load.'],1300,699,24);
    lines(['Full-run metric, calculated over 3–5 s.','Simulated performance, not a loaded robot test.'],1300,851,20,'#b5a78c',29);
  }else{
    board(result,scene==='lateral');
    const field=scene==='lateral'?f=>f.pose[1]*1000:f=>f.pose[1];
    chart(94,784,1104,128,scene==='lateral'?curves(field):[{color:color.eso,data:result.traces.eso.map(f=>[f.t,field(f)])}],scene==='lateral'?-150:-.65,scene==='lateral'?30:.45,scene==='lateral'?'Unwanted sideways drift · mm':'Body height · m');
    if(scene==='lateral'){
      text('LATERAL COORDINATION',1300,267,20,'#72debd',600);
      text('Maximum sideways drift',1300,320,28,'#e5edf5',500);
      variants.forEach((v,i)=>{text(names[v],1300,390+i*102,23,color[v]);text(`${(result.metrics[v].peak_cross_track_m*1000).toFixed(1)} mm`,1300,439+i*102,42,'#edf2f7',500);});
      lines(['Most improvement here comes from','synchronizing the wheel ramps.','Heading error remains; wheel feedback','alone is not full body-position control.'],1300,739,23,'#a5b8c8',33);
      text('Assumed contact physics; low-speed extrapolation.',1300,901,19,'#b5a78c');
    }else{
      text('TRACTION LIMIT',1300,267,20,'#e1a47b',600);
      text('At 4 s, surface friction drops',1300,320,28,'#e5edf5',500);
      text('0.65 → 0.078',1300,405,58,'#e1a47b',500);
      lines(['Assumed mass: 0.30 kg','Assumed total normal force: 12 N'],1300,467,24);
      text('0.94 N < 2.94 N',1300,606,47,'#edf2f7',500);
      lines(['Total friction upper bound < body weight','before allocating force across wheel directions.'],1300,649,20);
      lines(['Slip is detected using simulated body motion.','Available traction is still insufficient.','The trial ends at the lower board boundary.'],1300,770,23,'#a5b8c8',35);
      text('An estimator cannot create missing traction.',1300,901,21,'#e1a47b');
    }
  }
  text(`SIMULATION TIME  ${time.toFixed(2)} s`,67,986,21,'#72debd',500);
  text('Wheel dynamics: measured, suspended. Forces, contact and sensing: assumptions.',505,986,22,'#91a7bc');
  text('No claim of validated wall holding, adhesion safety, or deployed adaptive firmware.',67,1035,21,'#6f879e');
  c.fillStyle='#213747';c.fillRect(1530,1020,320,4);c.fillStyle='#72debd';c.fillRect(1530,1020,320*time/10,4);
}
window.portfolio={
  ready:true,
  renderAt:(which,t)=>{if(!results[which])throw Error('Unknown scene');scene=which;time=Math.max(0,Math.min(10,t));playing=false;render();},
  metadata:()=>({simulationOnly:true,model:'sysid-model-03.json',scenes:Object.fromEntries(Object.entries(results).map(([k,r])=>[k,r.metrics]))})
};
document.querySelectorAll('[data-scene]').forEach(b=>b.onclick=()=>{scene=b.dataset.scene;time=0;playing=false;render();});
document.getElementById('play').onclick=()=>{if(time>=10)time=0;playing=!playing;};
function animate(now){if(playing){time=Math.min(10,time+(now-before)/1000);if(time===10)playing=false;render();}before=now;requestAnimationFrame(animate);}render();requestAnimationFrame(animate);
