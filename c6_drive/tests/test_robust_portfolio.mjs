// Dedicated empty local Chrome profile only. No robot transport or personal tabs.
import assert from 'node:assert/strict';
import fs from 'node:fs';
const pages=await(await fetch('http://127.0.0.1:9238/json/list')).json();
const ws=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl),pending=new Map(),errors=[];let id=0;
await new Promise((r,j)=>{ws.addEventListener('open',r);ws.addEventListener('error',j);});
ws.addEventListener('message',e=>{const m=JSON.parse(e.data);if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails);if(m.id){const p=pending.get(m.id);if(p){clearTimeout(p.timer);pending.delete(m.id);m.error?p.reject(Error(JSON.stringify(m.error))):p.resolve(m.result);}}});
function call(method,params={}){return new Promise((resolve,reject)=>{const n=++id;pending.set(n,{resolve,reject,timer:setTimeout(()=>reject(Error(method+' timeout')),15000)});ws.send(JSON.stringify({id:n,method,params}));});}
async function js(expression){const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
const wait=ms=>new Promise(r=>setTimeout(r,ms));
async function until(expression){for(let n=0;n<150;n++){if(await js(expression))return;await wait(50);}throw Error('Timed out: '+expression);}
async function capture(name){const img=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});fs.writeFileSync(new URL('../reports/'+name,import.meta.url),Buffer.from(img.data,'base64'));}
try{
  await call('Runtime.enable');await call('Page.enable');
  await call('Emulation.setDeviceMetricsOverride',{width:1440,height:1050,deviceScaleFactor:1,mobile:false});
  await call('Page.navigate',{url:'http://127.0.0.1:8088/portfolio/robust.html'});
  await until(`document.getElementById('nominalRows')?.children.length===16`);
  assert.equal(await js(`document.documentElement.lang`),'en');
  assert.equal(await js(`document.getElementById('stressRows').children.length`),30);
  assert.equal(await js(`document.getElementById('scenario').options.length`),16);
  assert.equal(await js(`document.getElementById('reduction').textContent`),'52.8%');
  assert.equal(await js(`document.getElementById('falls').textContent`),'1/30');
  for(const url of await js(`Array.from(document.querySelectorAll('a[href]')).map(a=>a.href).filter(s=>s.startsWith(location.origin))`)){
    assert((await fetch(url,{method:'HEAD'})).ok,'Missing link '+url);
  }
  await capture('robust-gallery.png');
  const cases=await js(`Array.from(document.getElementById('scenario').options,o=>o.value)`);
  for(const name of cases){
    await js(`document.getElementById('scenario').value='${name}';document.getElementById('scenario').dispatchEvent(new Event('change'));document.getElementById('scrub').value=159;document.getElementById('scrub').dispatchEvent(new Event('input'));`);
    assert.equal(await js(`document.getElementById('time').textContent`),'7.96 s');
    assert((await js(`document.getElementById('scenarioNote').textContent`)).length>40);
    const expected={camera:'BODY LOST',encoder:'ENCODER INVALID',heartbeat:'COMMAND STALE',stuck:'FEEDBACK INCONSISTENT',slip:'EMERGENCY STOP',magnet:'EMERGENCY STOP'}[name];
    if(expected){assert.equal(await js(`document.getElementById('state').textContent`),expected);assert.equal(await js(`document.getElementById('pwm').textContent`),'+0 / +0 / +0');}
  }
  await js(`document.getElementById('scenario').value='saturation';document.getElementById('scenario').dispatchEvent(new Event('change'));document.getElementById('scrub').value=110;document.getElementById('scrub').dispatchEvent(new Event('input'));document.getElementById('scenario').scrollIntoView({block:'start'});`);
  await capture('robust-replay.png');
  await js(`document.getElementById('play').click()`);await wait(160);
  assert(await js(`Number(document.getElementById('scrub').value)>110`));
  await js(`document.getElementById('play').click()`);
  for(const name of ['floor','wall','saturation','slip']){
    await js(`document.getElementById('clip').value='${name}';document.getElementById('clip').dispatchEvent(new Event('change'));`);
    await until(`document.getElementById('video').readyState>=1`);
    assert.deepEqual(await js(`(()=>{const v=document.getElementById('video');return [v.videoWidth,v.videoHeight,Math.round(v.duration),v.error?.message||null]})()`),[1920,1080,10,null]);
  }
  await js(`document.getElementById('video').muted=true;document.getElementById('video').play()`);await wait(200);
  assert(await js(`document.getElementById('video').currentTime>0`));
  await js(`new Promise((resolve,reject)=>{const v=document.getElementById('video');v.pause();const t=setTimeout(()=>reject(Error('seek timed out')),4000);v.addEventListener('seeked',()=>{clearTimeout(t);resolve();},{once:true});v.currentTime=7;})`);
  assert.equal(await js(`document.getElementById('video').currentTime`),7);
  assert.equal(await js(`document.documentElement.scrollWidth>innerWidth`),false);
  await call('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});
  await js(`window.scrollTo(0,0)`);await wait(100);
  assert.equal(await js(`document.documentElement.scrollWidth>innerWidth`),false,'Mobile horizontal overflow');
  await capture('robust-mobile.png');
  assert.deepEqual(errors,[]);
  console.log('PASS: 16 physics replays, retained stress failures, all links, four 1080p clips, play/seek, desktop/mobile layout');
}finally{ws.close();}
