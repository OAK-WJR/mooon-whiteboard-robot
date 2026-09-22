// Inspect the existing local lab/gallery in the dedicated test Chrome profile.
// No request is made to the physical robot.
import assert from 'node:assert/strict';
import fs from 'node:fs';
const pages=await(await fetch('http://127.0.0.1:9238/json/list')).json();
const socket=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
await new Promise((r,j)=>{socket.addEventListener('open',r);socket.addEventListener('error',j);});
let id=0;const pending=new Map(),errors=[],requests=[];
socket.addEventListener('message',e=>{
 const m=JSON.parse(e.data);
 if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails);
 if(m.method==='Network.requestWillBeSent')requests.push(m.params.request.url);
 if(m.id&&pending.has(m.id)){const p=pending.get(m.id);pending.delete(m.id);clearTimeout(p.timer);m.error?p.reject(Error(JSON.stringify(m.error))):p.resolve(m.result);}
});
function call(method,params={}){return new Promise((resolve,reject)=>{const n=++id;pending.set(n,{resolve,reject,timer:setTimeout(()=>reject(Error(method+' timeout')),15000)});socket.send(JSON.stringify({id:n,method,params}));});}
async function js(expression){const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
const wait=ms=>new Promise(r=>setTimeout(r,ms));
const q=`document.querySelector('[data-encoder-results]')`;
const heavy=`document.querySelector('[data-load-results]')`;
try{
 await call('Page.enable');await call('Runtime.enable');await call('Network.enable');
 for(const path of ['sim','portfolio']){
  await call('Emulation.setDeviceMetricsOverride',{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  await call('Page.navigate',{url:`http://127.0.0.1:8088/${path}/#latest-results`});
  for(let n=0;n<100;n++){if(await js(`${q}?.dataset.loaded==='true'`))break;await wait(100);}
  assert.equal(await js(`${q}?.dataset.loaded`),'true','Recorded results did not load');
  for(let n=0;n<100;n++){if(await js(`${heavy}?.dataset.loaded==='true'`))break;await wait(100);}
  assert.equal(await js(`${heavy}?.dataset.loaded`),'true','Heavy-load results did not load');
  assert((await js(`${heavy}.querySelector('[data-observation]').textContent`)).includes(path==='sim'?'尚未称重':'weight not measured'));
  for(let n=0;n<100;n++){if(await js(`${heavy}.querySelector('[data-startup-results]')?.dataset.loaded==='true'`))break;await wait(100);}
  assert.equal(await js(`${heavy}.querySelector('[data-startup-results]')?.dataset.loaded`),'true','Startup experiment did not load');
  assert((await js(`${heavy}.querySelector('[data-startup-results]').textContent`)).includes('30'));
  assert.equal(await js(`${heavy}.dataset.case`),'1kg_high_drag_x_0.08');
  assert.equal(await js(`${heavy}.dataset.controller`),'load_servo');
  assert.equal(await js(`${heavy}.querySelectorAll('[data-validation] tbody tr')[0].children[2].textContent`),'17 / 36');
  const successorPlot=await js(`${heavy}.querySelector('[data-load-chart]').innerHTML`);
  await js(`{const s=${heavy}.querySelector('[data-controller]');s.value='fixed';s.dispatchEvent(new Event('change'));}`);
  assert.equal(await js(`${heavy}.dataset.controller`),'fixed');
  assert.notEqual(await js(`${heavy}.querySelector('[data-load-chart]').innerHTML`),successorPlot);
  await js(`{const s=${heavy}.querySelector('[data-controller]');s.value='load_servo';s.dispatchEvent(new Event('change'));}`);
  assert.equal(await js(`${heavy}.querySelectorAll('[data-load-chart] path').length`),4);
  assert.equal(await js(`${heavy}.querySelectorAll('[data-wheel-table] tbody tr').length`),3);
  const covered=await js(`(async()=>{
   const rows=(await(await fetch('/reports/heavy-load-summary.json')).json()).cases;
   const root=${heavy},visited=[];
   for(const row of rows){
    for(const key of ['motor_force_N','mass','profile','speed','axis']){
     const select=root.querySelector('[data-field="'+key+'"]');select.value=String(row.case[key]);select.dispatchEvent(new Event('change'));
    }
    if(root.dataset.case!==row.case.name)throw Error('Wrong case: '+row.case.name+' / '+root.dataset.case);
    if(root.querySelector('[data-load-chart]').innerHTML.includes('NaN'))throw Error('Nonfinite plot');
    visited.push(root.dataset.case);
   }
   return new Set(visited).size;
  })()`);
  assert.equal(covered,54,'All recorded mass/drag/speed/direction/torque choices are reachable');
  await js(`for(const [key,value] of Object.entries({motor_force_N:4,mass:1,profile:'high_drag',speed:.08,axis:0})){
   const s=${heavy}.querySelector('[data-field="'+key+'"]');s.value=String(value);s.dispatchEvent(new Event('change'));
  }${heavy}.scrollIntoView({block:'start'})`);
  let loadImage=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
  fs.writeFileSync(`c6_drive/reports/heavy-load-${path}-desktop.png`,Buffer.from(loadImage.data,'base64'));
  const loadLinks=await js(`Array.from(${heavy}.querySelectorAll('a')).map(a=>a.href)`);
  for(const link of loadLinks)assert((await fetch(link,{method:'HEAD'})).ok,link);
  assert.equal(await js(`${q}.querySelectorAll('tbody tr').length`),3);
  assert.equal(await js(`${q}.querySelectorAll('tbody tr')[1].children[2].textContent`),'24 / 24');
  assert.equal(await js(`${q}.querySelectorAll('tbody tr')[2].children[2].textContent`),'0 / 12');
  await js(`${q}.querySelector('img').decode()`);
  await js(`${q}.querySelector('[data-view="wall"]').click();${q}.querySelector('img').decode()`);
  assert((await js(`${q}.querySelector('img').src`)).endsWith('encoder-servo-wall.png'));
  await js(`${q}.querySelector('[data-view="video"]').click()`);
  for(let n=0;n<80;n++){if(await js(`${q}.querySelector('video').readyState>=1`))break;await wait(100);}
  assert.equal(await js(`${q}.querySelector('video').videoWidth`),1200);
  assert.equal(await js(`${q}.querySelector('video').videoHeight`),864);
  assert(Math.abs(await js(`${q}.querySelector('video').duration`)-10)<.1);
  await js(`${q}.querySelector('video').muted=true;${q}.querySelector('video').play()`);await wait(350);
  assert(await js(`${q}.querySelector('video').currentTime>0`));
  await js(`${q}.querySelector('video').pause();${q}.querySelector('video').currentTime=2.8`);await wait(350);
  assert(await js(`${q}.querySelector('video').currentTime>=2.8`));
  await js(`${q}.querySelector('video').scrollIntoView({block:'center'})`);
  let img=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
  fs.writeFileSync(`c6_drive/reports/encoder-results-${path}-desktop.png`,Buffer.from(img.data,'base64'));
  const links=await js(`Array.from(${q}.querySelectorAll('a')).map(a=>a.href)`);
  for(const link of links)assert((await fetch(link,{method:'HEAD'})).ok,link);
  assert.equal(await js(`document.documentElement.scrollWidth>innerWidth`),false);
  await call('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});
  await js(`${q}.scrollIntoView({block:'start'})`);await wait(200);
  assert.equal(await js(`document.documentElement.scrollWidth>innerWidth`),false,`Mobile overflow ${path}`);
  img=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
  fs.writeFileSync(`c6_drive/reports/encoder-results-${path}-mobile.png`,Buffer.from(img.data,'base64'));
  await js(`${heavy}.scrollIntoView({block:'start'})`);await wait(100);
  assert.equal(await js(`document.documentElement.scrollWidth>innerWidth`),false,`Load panel mobile overflow ${path}`);
  loadImage=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
  fs.writeFileSync(`c6_drive/reports/heavy-load-${path}-mobile.png`,Buffer.from(loadImage.data,'base64'));
 }
 assert.deepEqual(errors,[]);
 assert(requests.every(url=>url.startsWith('http://127.0.0.1:8088/')||url.startsWith('data:')),'Unexpected external or robot request');
 console.log('PASS: lab/gallery, all 54 heavy-load selections, charts, video play/seek, links, mobile/desktop, no robot requests');
}finally{socket.close();}
