// Dedicated empty Chrome profile only; inspect local media, never the live robot.
import assert from 'node:assert/strict';
import fs from 'node:fs';
const pages=await(await fetch('http://127.0.0.1:9238/json/list')).json();
const ws=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl),pending=new Map(),errors=[];let id=0;
await new Promise((r,j)=>{ws.addEventListener('open',r);ws.addEventListener('error',j);});
ws.addEventListener('message',e=>{const m=JSON.parse(e.data);if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails);if(m.id){const p=pending.get(m.id);if(p){clearTimeout(p.timer);pending.delete(m.id);m.error?p.reject(Error(JSON.stringify(m.error))):p.resolve(m.result);}}});
function call(method,params={}){return new Promise((resolve,reject)=>{const n=++id;pending.set(n,{resolve,reject,timer:setTimeout(()=>reject(Error(method+' timeout')),15000)});ws.send(JSON.stringify({id:n,method,params}));});}
async function js(expression){const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
const wait=ms=>new Promise(r=>setTimeout(r,ms));
try{
  await call('Runtime.enable');await call('Page.enable');
  await call('Emulation.setDeviceMetricsOverride',{width:1440,height:1000,deviceScaleFactor:1,mobile:false});
  await call('Page.navigate',{url:'http://127.0.0.1:8088/portfolio/'});
  for(let n=0;n<100;n++){if(await js(`document.getElementById('metrics')?.children.length===12`))break;await wait(100);}
  assert.equal(await js(`document.querySelectorAll('#metrics tr').length`),12);
  assert.equal(await js(`document.getElementById('scenario').options.length`),12);
  assert.equal(await js(`document.documentElement.lang`),'en');
  const files=await js(`Array.from(document.querySelectorAll('a[href]')).map(a=>a.href).filter(s=>s.startsWith(location.origin))`);
  for(const url of files){const r=await fetch(url,{method:'HEAD'});assert(r.ok,'Missing gallery link '+url);}
  const media='http://127.0.0.1:8088/portfolio/mujoco-wall.mp4';
  const partial=await fetch(media,{headers:{Range:'bytes=16-47'}});
  assert.equal(partial.status,206);assert.equal((await partial.arrayBuffer()).byteLength,32);
  const suffix=await fetch(media,{headers:{Range:'bytes=-16'}});
  assert.equal(suffix.status,206);assert.equal((await suffix.arrayBuffer()).byteLength,16);
  assert.equal((await fetch(media,{headers:{Range:'bytes=999999999-'}})).status,416);
  for(const name of ['wall','slip','load','saturation','camera','encoder','magnet','floor','payload','weak','sag','slope']){
    await js(`document.getElementById('scenario').value='${name}';document.getElementById('scenario').dispatchEvent(new Event('change'));`);
    for(let n=0;n<60;n++){if(await js(`document.getElementById('physicsVideo').readyState>=1`))break;await wait(50);}
    const video=await js(`(()=>{const v=document.getElementById('physicsVideo');return {error:v.error?.message,duration:v.duration,w:v.videoWidth,h:v.videoHeight}})()`);
    assert(!video.error);assert.equal(video.w,1920);assert.equal(video.h,1080);assert(Math.abs(video.duration-10)<.1);
  }
  await js(`document.getElementById('physicsVideo').muted=true;document.getElementById('physicsVideo').play()`);await wait(300);
  assert(await js(`document.getElementById('physicsVideo').currentTime>0`));
  await js(`new Promise((resolve,reject)=>{const v=document.getElementById('physicsVideo');v.pause();
    const timer=setTimeout(()=>reject(Error('Seek timeout')),4000);
    v.addEventListener('seeked',()=>{clearTimeout(timer);resolve();},{once:true});v.currentTime=5;})`);
  assert(await js(`document.getElementById('physicsVideo').currentTime>=5`));
  const image=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
  fs.writeFileSync(new URL('../reports/portfolio-gallery.png',import.meta.url),Buffer.from(image.data,'base64'));
  assert.equal(await js(`document.documentElement.scrollWidth>innerWidth`),false);
  await call('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await wait(100);
  assert.equal(await js(`document.documentElement.scrollWidth>innerWidth`),false,'Mobile overflow');
  assert.deepEqual(errors,[]);
  console.log('PASS: English gallery, 12 scenarios, all links, video metadata/play/seek, desktop/mobile layout');
}finally{ws.close();}
