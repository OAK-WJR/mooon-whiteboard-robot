import {Simulation,modelsFrom,defaults,presets,variants,labels,DT,R,wheelMap} from './core.mjs';
const $=id=>document.getElementById(id),colors={legacy:'#d89568',model:'#76a9ed',eso:'#70dbb3'};
const statusText={RUNNING:'运行中',SIMULATED_HOLD:'位置闭环',SATURATED:'输出饱和',SLIP_DETECTED:'检测到滑移',
  POSITION_FEEDBACK_LOST:'位置反馈丢失',ENCODER_INVALID:'编码器失效',FELL_OFF_BOARD:'跌出板面 · 轨迹终止',OUTPUT_OFF:'驱动力已撤除'};
const notes={load:'目标 5200 counts/s；3 秒加入阻力，7 秒撤除。虚线为目标轮速。',
  mismatch:'真实仿真对象的增益为基准的 75%、响应时间为 140%；控制器不知道这些变化。',
  saturation:'默认供电比例 75%、额外阻力 145 PWM。补偿有上限，动力不足时仍会失速。',
  encoder:'第 5 秒模拟编码器失效。三组输出均归零，不继续使用无效反馈。',
  wall:'第 1–3 秒上行，随后松键。候选组使用模拟位置反馈保持位置；现有 PI 组松键滑行。',
  slip:'第 4 秒摩擦降至设定值的 12%。吸附牵引不足时，算法也无法阻止下滑。',
  camera:'第 4 秒停止位置测量；候选组反馈超时后撤掉输出，随后可能下滑。这不是实物防坠方案。',
  lateral:'水平面、无重力切向分量。先右移，再左移；默认关闭外部位置反馈，观察三轮响应差导致的偏移。可切换三轮同步加速。',
  manual:'点击开始，然后使用 WASD / Q、E。WASD 按白板坐标移动，松键可比较驻坡与滑行。'};
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
  $('run').textContent='开始仿真';$('scenarioNote').textContent=notes[options.preset];
  $('benchSettings').hidden=options.mode!=='bench';$('wallSettings').hidden=options.mode!=='wall';
  $('keys').hidden=options.preset!=='manual';
  $('stageTitle').textContent=options.mode==='bench'?'轮速响应':options.preset==='lateral'?'左右往返 · 三轮协同响应':'白板上的车体运动（假设模型）';
  $('plotTitle').textContent=options.mode==='bench'?'电机 PWM':options.preset==='lateral'?'横向偏移 · m':'车体高度 · m';
  $('errorLabel').textContent=options.mode==='bench'?'ESO · 轮速误差':options.preset==='lateral'?'ESO · 横向偏移':'ESO · 位置误差';
  $('metricHead').textContent=options.mode==='bench'?'3–5秒平均误差':options.preset==='lateral'?'最大横向偏移 / 航向差':'当前位置误差';
  draw();
}
function presetChanged(){
  const o={...defaults,...presets[$('preset').value]};
  for(const id of ['load','mass','normal','mu','force','speed','voltage'])$(id).value=o[id];
  $('camera').checked=o.camera;$('hold').checked=o.hold;$('sync').checked=o.sync;refreshValues();reset();
}
function pause(){running=false;keys.clear();paintKeys();$('run').textContent='继续仿真';draw();}
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
  if(!all.length){c.fillStyle='#6b849b';c.textAlign='center';c.fillText('开始实验后显示曲线',w/2,h/2);}
}
function board(){
  const [c,w,h]=context('stage'),scale=Math.min((w-90)/1.4,(h-38)/1.3),cx=w/2,cy=h-18-.60*scale;
  const X=x=>cx+x*scale,Y=y=>cy-y*scale;
  c.fillStyle='#101923';c.fillRect(X(-.7),Y(.65),1.4*scale,1.25*scale);
  c.strokeStyle='#293847';c.lineWidth=1;
  for(let x=-.6;x<=.61;x+=.2){c.beginPath();c.moveTo(X(x),Y(-.6));c.lineTo(X(x),Y(.65));c.stroke();}
  for(let y=-.6;y<=.65;y+=.2){c.beginPath();c.moveTo(X(-.7),Y(y));c.lineTo(X(.7),Y(y));c.stroke();}
  c.fillStyle='#7990a5';c.font='10px system-ui';c.textAlign='left';c.fillText(options.preset==='lateral'?'水平面 · 网格 20cm':'↑ 竖直板面 · 网格 20cm',12,12);
  if(options.preset!=='lateral'){c.fillStyle='#c38768';c.fillText('↓ 重力',12,32);}
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
  graph('observerPlot',observer,{unit:options.mode==='bench'?'PWM · 虚线为已知注入值':'PWM 等效值 · 无实测扭矩'});
  $('time').innerHTML=`${(frame?.t??0).toFixed(2)} <em>s</em>`;
  $('comp').innerHTML=`${(frame?.compensation[motor]??0).toFixed(1)} <em>PWM</em>`;
  const err=frame?(options.mode==='bench'?Math.abs(frame.targets[motor]-frame.speed[motor]):options.preset==='lateral'?1000*Math.abs(frame.pose[1]):1000*Math.hypot(frame.pose[0]-frame.goal[0],frame.pose[1]-frame.goal[1])):null;
  $('error').innerHTML=err===null?'—':`${err.toFixed(options.mode==='bench'?0:1)} <em>${options.mode==='bench'?'counts/s':'mm'}</em>`;
  $('state').textContent=frame?(running?'运行 · ':'暂停 · ')+(statusText[frame.status]??frame.status):'准备就绪';
  $('results').innerHTML=variants.map(v=>{
    const data=traces[v],f=data.at(-1),window=data.filter(f=>f.t>=3&&f.t<5);
    const mae=window.length?window.reduce((a,f)=>a+Math.abs(f.speed[motor]-f.targets[motor]),0)/window.length:null;
    const value=options.mode==='bench'?(mae===null?'—':mae.toFixed(0)+' counts/s'):options.preset==='lateral'?
      (f?`${(1000*Math.max(...data.map(f=>Math.abs(f.pose[1])))).toFixed(1)} mm / ${(Math.max(...data.map(f=>Math.abs(f.pose[2]-f.goal[2])))*180/Math.PI).toFixed(1)}°`:'—'):
      (f?(1000*Math.hypot(f.pose[0]-f.goal[0],f.pose[1]-f.goal[1])).toFixed(1)+' mm':'—');
    return `<tr><td style="color:${colors[v]}">${labels[v]}</td><td>${f?statusText[f.status]:'待开始'}</td><td>${data.length?Math.max(...data.flatMap(f=>f.pwm.map(Math.abs))):'—'}</td><td>${value}</td></tr>`;
  }).join('');
  $('summary').textContent=options.mode==='bench'?'比较蓝线与绿线可单独观察 ESO 的作用；两组 PI 与前馈相同。红线为原轮速算法。误差指标只统计加入负载后的 3–5 秒。':options.preset==='lateral'?
    `这是水平面左右往返；三轮同步加速${options.sync?'已启用':'已关闭'}，候选组位置反馈${options.camera?'已启用':'已关闭'}。表中显示整段轨迹的最大横向偏移和航向差，均为假设模型结果。`:
    '蓝、绿组额外使用模拟位置反馈和可选 HOLD，红组只有原轮速算法。驻坡差异不能全部归功于 ESO；跌出板面后终止该组轨迹。';
}
function animate(now){
  const elapsed=Math.min(.1,(now-lastTime)/1000||0);lastTime=now;
  if(running){accumulator+=elapsed;while(accumulator>=DT&&running){tick();accumulator-=DT;}draw();}
  requestAnimationFrame(animate);
}
async function init(){
  try{
    const [m,g]=await Promise.all([fetch('../reports/sysid-model-03.json'),fetch('../calibration.json')]);
    if(!m.ok||!g.ok)throw Error('模型文件读取失败');models=modelsFrom(await m.json());gains=(await g.json()).gains;
    $('preset').onchange=presetChanged;
    for(const id of ['motor','direction','load','mass','normal','mu','force','speed','voltage','camera','hold','sync'])
      $(id).addEventListener('input',()=>{refreshValues();reset();});
    $('reset').onclick=reset;
    $('run').onclick=()=>{if(running)pause();else{if(sims.eso.stopped||sims.eso.t>=10&&options.preset!=='manual')reset();running=true;$('run').textContent='暂停';}};
    $('off').onclick=()=>{keys.clear();paintKeys();variants.forEach(v=>sims[v].stop());running=true;$('run').textContent='暂停';};
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
  }catch(e){$('state').textContent='无法加载';$('summary').textContent=e.message;$('run').disabled=true;}
}
init();
