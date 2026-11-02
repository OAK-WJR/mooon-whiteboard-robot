import {Simulation,modelsFrom,defaults,presets,variants,labels,DT,R,wheelMap} from './core.mjs';
const $=id=>document.getElementById(id),colors={legacy:'#d89568',model:'#76a9ed',eso:'#70dbb3'};
const statusText={RUNNING:'Running',SIMULATED_HOLD:'Position closed-loop',SATURATED:'Output saturated',SLIP_DETECTED:'Slip detected',
  POSITION_FEEDBACK_LOST:'Position feedback lost',ENCODER_INVALID:'Encoder failed',FELL_OFF_BOARD:'Fell off the board · track ended',OUTPUT_OFF:'Drive force removed'};
const notes={load:'Target 5200 counts/s; resistance added at 3 s and removed at 7 s. The dashed line is the target wheel speed.',
  mismatch:'The simulated plant has 75% of the baseline gain and 140% of the response time; the controllers do not know about these changes.',
  saturation:'Default supply ratio 75%, extra resistance 145 PWM. Compensation is capped, so with too little power the wheels still stall.',
  encoder:'Encoder failure simulated at 5 s. All three groups set output to zero and stop using the invalid feedback.',
  wall:'Climb from 1–3 s, then release the keys. The candidate groups hold position with simulated position feedback; the existing PI group coasts after release.',
  slip:'At 4 s friction drops to 12% of the set value. When adhesion traction is too low, the algorithms cannot stop the slide either.',
  camera:'Position measurement stops at 4 s; the candidate groups remove output after the feedback times out and may then slide down. This is not a fall-prevention design for the real robot.',
  lateral:'Level surface, no tangential gravity component. Move right, then left; external position feedback is off by default, to show the drift caused by response differences between the three wheels. Three-wheel synchronized acceleration can be toggled.',
  manual:'Click Start, then use WASD / Q, E. WASD moves in whiteboard coordinates; release the keys to compare holding on the slope with coasting.'};
let models,gains,sims={},traces={},options={...defaults},running=false,accumulator=0,lastTime=0;
const keys=new Set();
function refreshValues(){
  const formats={load:x=>`${x} PWM`,mass:x=>`${Number(x).toFixed(2)} kg`,normal:x=>`${x} N`,
    mu:x=>Number(x).toFixed(2),force:x=>`${Number(x).toFixed(1)} N`,speed:x=>`${Number(x).toFixed(2)} m/s`,voltage:x=>`${Math.round(Number(x)*100)}%`};
  for(const [id,fmt] of Object.entries(formats))$(id+'Value').textContent=fmt($(id).value);
}
function readOptions(){
  const preset=$('preset').value;
  const o={...defaults,...presets[preset],preset};
  for(const id of ['load','mass','normal','mu','force','speed','voltage','motor','direction'])o[id]=Number($(id).value);
  o.camera=$('camera').checked;o.hold=$('hold').checked;o.sync=$('sync').checked;return o;
}
function reset(){
  running=false;accumulator=0;keys.clear();paintKeys();options=readOptions();
  sims=Object.fromEntries(variants.map(v=>[v,new Simulation(models,gains,v,options)]));
  traces=Object.fromEntries(variants.map(v=>[v,[]]));
  $('run').textContent='Start simulation';$('scenarioNote').textContent=notes[options.preset];
  $('benchSettings').hidden=options.mode!=='bench';$('wallSettings').hidden=options.mode!=='wall';
  $('keys').hidden=options.preset!=='manual';
  $('stageTitle').textContent=options.mode==='bench'?'Wheel-speed response':options.preset==='lateral'?'Left-right round trip · coordinated three-wheel response':'Robot motion on the whiteboard (assumed model)';
  $('plotTitle').textContent=options.mode==='bench'?'Motor PWM':options.preset==='lateral'?'Lateral drift · m':'Body height · m';
  $('errorLabel').textContent=options.mode==='bench'?'ESO · wheel-speed error':options.preset==='lateral'?'ESO · lateral drift':'ESO · position error';
  $('metricHead').textContent=options.mode==='bench'?'Mean error 3–5 s':options.preset==='lateral'?'Max lateral drift / heading difference':'Current position error';
  draw();
}
function presetChanged(){
  const o={...defaults,...presets[$('preset').value]};
  for(const id of ['load','mass','normal','mu','force','speed','voltage'])$(id).value=o[id];
  $('camera').checked=o.camera;$('hold').checked=o.hold;$('sync').checked=o.sync;refreshValues();reset();
}
function pause(){running=false;keys.clear();paintKeys();$('run').textContent='Resume simulation';draw();}
function paintKeys(){document.querySelectorAll('[data-key]').forEach(b=>b.classList.toggle('active',keys.has(b.dataset.key)));}
function command(){
  let x=(keys.has('d')-keys.has('a'))*options.speed,y=(keys.has('w')-keys.has('s'))*options.speed;
  const n=Math.hypot(x,y);if(n>options.speed){x*=options.speed/n;y*=options.speed/n;}
  return [x,y,(keys.has('q')-keys.has('e'))*1.2];
}
function tick(){
  const input=options.preset==='manual'?command():null;
  for(const v of variants)traces[v].push(sims[v].step(input));
  if(sims.eso.t>=(options.preset==='manual'?60:10))pause();
}
function context(id){
  const canvas=$(id),rect=canvas.getBoundingClientRect(),dpr=devicePixelRatio||1;
  if(canvas.width!==Math.round(rect.width*dpr)||canvas.height!==Math.round(rect.height*dpr)){
    canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);
  }
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,rect.width,rect.height);
  return [ctx,rect.width,rect.height];
}
function graph(id,curves,{range=null,unit='',target=null}={}){
  const [c,w,h]=context(id),left=54,right=12,top=16,bottom=28;
  const all=curves.flatMap(x=>x.data.map(p=>p[1]));if(target!==null)all.push(target);
  let lo=range?.[0]??Math.min(0,...all),hi=range?.[1]??Math.max(1,...all);
  if(!range){const margin=(hi-lo)*.12;lo-=margin;hi+=margin;}
  const end=Math.max(10,sims.eso?.t??0),begin=Math.max(0,end-15);
  const X=t=>left+(t-begin)/(end-begin)*(w-left-right),Y=v=>h-bottom-(v-lo)/(hi-lo)*(h-top-bottom);
  c.font='10px system-ui';c.textBaseline='middle';
  for(let k=0;k<=4;k++){
    const y=lo+(hi-lo)*k/4,py=Y(y);c.strokeStyle='#283340';c.lineWidth=1;c.beginPath();c.moveTo(left,py);c.lineTo(w-right,py);c.stroke();
    c.fillStyle='#7890a5';c.textAlign='right';c.fillText(Math.abs(y)>=100?Math.round(y).toString():y.toFixed(1),left-8,py);
  }
  c.textAlign='center';for(let k=0;k<=5;k++){const t=begin+(end-begin)*k/5;c.fillText(t.toFixed(0),X(t),h-10);}
  c.textAlign='left';c.fillText(unit,left,5);
  if(target!==null){c.setLineDash([4,4]);c.strokeStyle='#65798b';c.beginPath();c.moveTo(left,Y(target));c.lineTo(w-right,Y(target));c.stroke();c.setLineDash([]);}
  c.save();c.beginPath();c.rect(left,top,w-left-right,h-top-bottom);c.clip();
  for(const curve of curves){
    c.strokeStyle=curve.color;c.lineWidth=1.6;c.setLineDash(curve.dash??[]);c.beginPath();let first=true;
    for(const [t,y] of curve.data){if(t<begin)continue;if(first){c.moveTo(X(t),Y(y));first=false;}else c.lineTo(X(t),Y(y));}c.stroke();
  }c.restore();c.setLineDash([]);
  if(!all.length){c.fillStyle='#6b849b';c.textAlign='center';c.fillText('Curves appear after the experiment starts',w/2,h/2);}
}
function board(){
  const [c,w,h]=context('stage'),scale=Math.min((w-90)/1.4,(h-38)/1.3),cx=w/2,cy=h-18-.60*scale;
  const X=x=>cx+x*scale,Y=y=>cy-y*scale;
  c.fillStyle='#101923';c.fillRect(X(-.7),Y(.65),1.4*scale,1.25*scale);
  c.strokeStyle='#293847';c.lineWidth=1;
  for(let x=-.6;x<=.61;x+=.2){c.beginPath();c.moveTo(X(x),Y(-.6));c.lineTo(X(x),Y(.65));c.stroke();}
  for(let y=-.6;y<=.65;y+=.2){c.beginPath();c.moveTo(X(-.7),Y(y));c.lineTo(X(.7),Y(y));c.stroke();}
  c.fillStyle='#7990a5';c.font='10px system-ui';c.textAlign='left';c.fillText(options.preset==='lateral'?'Level surface · grid 20cm':'↑ Vertical board · grid 20cm',12,12);
  if(options.preset!=='lateral'){c.fillStyle='#c38768';c.fillText('↓ Gravity',12,32);}
  c.setLineDash([5,4]);c.strokeStyle='#8f535e';c.beginPath();c.moveTo(X(-.7),Y(-.6));c.lineTo(X(.7),Y(-.6));c.stroke();c.setLineDash([]);
  for(const v of variants){
    const data=traces[v],f=data.at(-1),pose=f?.pose??[0,0,Math.PI/2],color=colors[v];
    c.strokeStyle=color;c.lineWidth=1.3;c.globalAlpha=.6;c.beginPath();
    data.filter((_,i)=>i%5===0).forEach((p,i)=>{if(!i)c.moveTo(X(p.pose[0]),Y(p.pose[1]));else c.lineTo(X(p.pose[0]),Y(p.pose[1]));});c.stroke();c.globalAlpha=1;
    const [x,y,theta]=pose;c.save();c.translate(X(x),Y(y));c.rotate(-theta);
    c.fillStyle=color+'25';c.strokeStyle=color;c.lineWidth=2;c.beginPath();c.moveTo(17,0);c.lineTo(-11,13);c.lineTo(-11,-13);c.closePath();c.fill();c.stroke();
    for(const a of wheelMap(0)){const px=R*a[1]*scale,py=R*a[0]*scale;c.fillStyle=color;c.fillRect(px-3,py-3,6,6);}
    c.restore();
    c.fillStyle=color;c.font='10px system-ui';c.fillText(v==='legacy'?'PI':v==='model'?'MODEL':'ESO',X(x)+21,Y(y)+(v==='legacy'?16:v==='model'?0:-16));
  }
  const goal=sims.eso?.goal;if(goal){c.strokeStyle='#d6dee6';c.setLineDash([3,4]);c.beginPath();c.arc(X(goal[0]),Y(goal[1]),7,0,Math.PI*2);c.stroke();c.setLineDash([]);}
}
function draw(){
  if(!models)return;
  const motor=options.motor,frame=traces.eso.at(-1);
  const curves=field=>variants.map(v=>({color:colors[v],data:traces[v].filter((_,i)=>i%2===0).map(f=>[f.t,field(f)])}));
  if(options.mode==='bench')graph('stage',curves(f=>f.speed[motor]),{target:options.direction*5200,unit:'counts/s'});else board();
  graph('plot',curves(options.mode==='bench'?f=>f.pwm[motor]:f=>f.pose[1]),{range:options.mode==='bench'?[-260,260]:null,unit:options.mode==='bench'?'PWM':'m'});
  const observer=[{color:colors.eso,data:traces.eso.map(f=>[f.t,f.compensation[motor]])}];
  if(options.mode==='bench')observer.push({color:'#70859a',dash:[4,4],data:traces.eso.map(f=>[f.t,f.load*options.direction])});
  graph('observerPlot',observer,{unit:options.mode==='bench'?'PWM · dashed line is the known injected value':'PWM equivalent · no measured torque'});
  $('time').innerHTML=`${(frame?.t??0).toFixed(2)} <em>s</em>`;
  $('comp').innerHTML=`${(frame?.compensation[motor]??0).toFixed(1)} <em>PWM</em>`;
  const err=frame?(options.mode==='bench'?Math.abs(frame.targets[motor]-frame.speed[motor]):options.preset==='lateral'?1000*Math.abs(frame.pose[1]):1000*Math.hypot(frame.pose[0]-frame.goal[0],frame.pose[1]-frame.goal[1])):null;
  $('error').innerHTML=err===null?'—':`${err.toFixed(options.mode==='bench'?0:1)} <em>${options.mode==='bench'?'counts/s':'mm'}</em>`;
  $('state').textContent=frame?(running?'Running · ':'Paused · ')+(statusText[frame.status]??frame.status):'Ready';
  $('results').innerHTML=variants.map(v=>{
    const data=traces[v],f=data.at(-1),window=data.filter(f=>f.t>=3&&f.t<5);
    const mae=window.length?window.reduce((a,f)=>a+Math.abs(f.speed[motor]-f.targets[motor]),0)/window.length:null;
    const value=options.mode==='bench'?(mae===null?'—':mae.toFixed(0)+' counts/s'):options.preset==='lateral'?
      (f?`${(1000*Math.max(...data.map(f=>Math.abs(f.pose[1])))).toFixed(1)} mm / ${(Math.max(...data.map(f=>Math.abs(f.pose[2]-f.goal[2])))*180/Math.PI).toFixed(1)}°`:'—'):
      (f?(1000*Math.hypot(f.pose[0]-f.goal[0],f.pose[1]-f.goal[1])).toFixed(1)+' mm':'—');
    return `<tr><td style="color:${colors[v]}">${labels[v]}</td><td>${f?statusText[f.status]:'Not started'}</td><td>${data.length?Math.max(...data.flatMap(f=>f.pwm.map(Math.abs))):'—'}</td><td>${value}</td></tr>`;
  }).join('');
  $('summary').textContent=options.mode==='bench'?'Compare the blue and green lines to see the effect of ESO alone; both groups share the same PI and feedforward. The red line is the original wheel-speed algorithm. The error metric only counts 3–5 s after the load is added.':options.preset==='lateral'?
    `This is a left-right round trip on a level surface; three-wheel synchronized acceleration is ${options.sync?'on':'off'}, candidate position feedback is ${options.camera?'on':'off'}. The table shows the maximum lateral drift and heading difference over the whole track; all are results of the assumed model.`:
    'The blue and green groups also use simulated position feedback and optional HOLD; the red group has only the original wheel-speed algorithm. Differences in holding on the slope cannot all be credited to ESO; a group track ends once it falls off the board.';
}
function animate(now){
  const elapsed=Math.min(.1,(now-lastTime)/1000||0);lastTime=now;
  if(running){accumulator+=elapsed;while(accumulator>=DT&&running){tick();accumulator-=DT;}draw();}
  requestAnimationFrame(animate);
}
async function init(){
  try{
    const [m,g]=await Promise.all([fetch('../reports/sysid-model-03.json'),fetch('../calibration.json')]);
    if(!m.ok||!g.ok)throw Error('Failed to read the model files');models=modelsFrom(await m.json());gains=(await g.json()).gains;
    $('preset').onchange=presetChanged;
    for(const id of ['motor','direction','load','mass','normal','mu','force','speed','voltage','camera','hold','sync'])
      $(id).addEventListener('input',()=>{refreshValues();reset();});
    $('reset').onclick=reset;
    $('run').onclick=()=>{if(running)pause();else{if(sims.eso.stopped||sims.eso.t>=10&&options.preset!=='manual')reset();running=true;$('run').textContent='Pause';}};
    $('off').onclick=()=>{keys.clear();paintKeys();variants.forEach(v=>sims[v].stop());running=true;$('run').textContent='Pause';};
    $('export').onclick=()=>{
      const data={schema:'wbb-browser-simulation-1',options,simulation_only:true,model_source:'sysid-model-03.json',traces};
      const url=URL.createObjectURL(new Blob([JSON.stringify(data)],{type:'application/json'}));
      const link=document.createElement('a');link.href=url;link.download='wbb-simulation.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    };
    window.addEventListener('keydown',e=>{
      if(e.key==='Escape'){e.preventDefault();$('off').click();return;}
      if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName)||e.target.isContentEditable||e.ctrlKey||e.metaKey||e.altKey)return;
      const key=e.key.toLowerCase();if('wasdqe'.includes(key)&&key.length===1&&running&&options.preset==='manual'){
        e.preventDefault();keys.add(key);paintKeys();
      }
    });
    window.addEventListener('keyup',e=>{keys.delete(e.key.toLowerCase());paintKeys();});
    window.addEventListener('blur',pause);document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
    document.querySelectorAll('[data-key]').forEach(b=>{
      b.onpointerdown=e=>{e.preventDefault();if(!running)return;b.setPointerCapture(e.pointerId);keys.add(b.dataset.key);paintKeys();};
      b.onpointerup=b.onpointercancel=b.onlostpointercapture=()=>{keys.delete(b.dataset.key);paintKeys();};
    });
    new ResizeObserver(draw).observe($('stage'));refreshValues();reset();requestAnimationFrame(animate);
  }catch(e){$('state').textContent='Failed to load';$('summary').textContent=e.message;$('run').disabled=true;}
}
init();
