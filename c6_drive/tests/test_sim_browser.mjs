// Run against a dedicated, empty Chrome --headless profile on port 9238.
// Tests only the local simulator, never the robot IP or serial interface.
import assert from 'node:assert/strict';
import fs from 'node:fs';
const pages=await(await fetch('http://127.0.0.1:9238/json/list')).json();
const page=pages.find(p=>p.type==='page');if(!page)throw Error('Dedicated test browser missing');
const ws=new WebSocket(page.webSocketDebuggerUrl),pending=new Map(),errors=[];let id=0;
await new Promise((resolve,reject)=>{ws.addEventListener('open',resolve);ws.addEventListener('error',reject);});
ws.addEventListener('message',e=>{
  const msg=JSON.parse(e.data);
  if(msg.method==='Runtime.exceptionThrown')errors.push(msg.params.exceptionDetails);
  if(msg.id){const p=pending.get(msg.id);if(p){clearTimeout(p.timer);pending.delete(msg.id);msg.error?p.reject(Error(JSON.stringify(msg.error))):p.resolve(msg.result);}}
});
function call(method,params={}){return new Promise((resolve,reject)=>{const n=++id;pending.set(n,{resolve,reject,timer:setTimeout(()=>reject(Error('CDP timeout '+method)),10000)});ws.send(JSON.stringify({id:n,method,params}));});}
async function evaluate(expression){const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
const wait=ms=>new Promise(r=>setTimeout(r,ms));
async function select(value){await evaluate(`document.getElementById('preset').value='${value}';document.getElementById('preset').dispatchEvent(new Event('change'));`);}
async function snapshot(){return evaluate(`(async()=>{
  const original=URL.createObjectURL,click=HTMLAnchorElement.prototype.click;let capture;
  URL.createObjectURL=b=>{capture=b.text();return original(b);};HTMLAnchorElement.prototype.click=()=>{};
  document.getElementById('export').click();URL.createObjectURL=original;HTMLAnchorElement.prototype.click=click;
  return JSON.parse(await capture);
})()`);}
try{
  await call('Runtime.enable');await call('Page.enable');
  await call('Emulation.setDeviceMetricsOverride',{width:1440,height:1200,deviceScaleFactor:1,mobile:false});
  await call('Page.navigate',{url:'http://127.0.0.1:8088/sim/'});
  for(let n=0;n<60;n++){if(await evaluate(`!!document.getElementById('results')?.children.length`))break;await wait(100);}
  assert.equal(await evaluate(`document.getElementById('run').disabled`),false);
  await evaluate(`document.getElementById('run').click()`);await wait(700);
  assert((await snapshot()).traces.eso.length>30);
  await evaluate(`document.getElementById('run').click()`);
  const paused=(await snapshot()).traces.eso.length;await wait(200);assert.equal((await snapshot()).traces.eso.length,paused);
  await select('lateral');assert.equal(await evaluate(`document.getElementById('camera').checked`),false);
  await select('manual');
  // Focused Start button must not swallow WASD.
  await evaluate(`document.getElementById('run').focus();document.getElementById('run').click()`);
  await call('Input.dispatchKeyEvent',{type:'keyDown',key:'w',code:'KeyW',windowsVirtualKeyCode:87});await wait(850);
  assert(await evaluate(`document.querySelector('[data-key="w"]').classList.contains('active')`));
  let data=await snapshot();assert(data.traces.eso.at(-1).goal[1]>.08,'W must move the simulated goal upward');
  await call('Input.dispatchKeyEvent',{type:'keyUp',key:'w',code:'KeyW',windowsVirtualKeyCode:87});
  assert.equal(await evaluate(`document.querySelector('[data-key="w"]').classList.contains('active')`),false);
  await evaluate(`document.getElementById('mass').focus()`);
  await call('Input.dispatchKeyEvent',{type:'keyDown',key:'a',code:'KeyA',windowsVirtualKeyCode:65});
  assert.equal(await evaluate(`document.querySelector('[data-key="a"]').classList.contains('active')`),false);
  await call('Input.dispatchKeyEvent',{type:'keyUp',key:'a',code:'KeyA',windowsVirtualKeyCode:65});
  await call('Input.dispatchKeyEvent',{type:'keyDown',key:'Escape',code:'Escape',windowsVirtualKeyCode:27});await wait(80);
  data=await snapshot();assert(data.traces.eso.at(-1).pwm.every(x=>x===0));
  await evaluate(`window.dispatchEvent(new Event('blur'))`);
  const blurred=(await snapshot()).traces.eso.length;await wait(150);assert.equal((await snapshot()).traces.eso.length,blurred);
  await select('load');await evaluate(`document.getElementById('run').click()`);await wait(3800);await evaluate(`document.getElementById('run').click()`);
  const image=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:true});
  fs.writeFileSync(new URL('../reports/simulation-ui.png',import.meta.url),Buffer.from(image.data,'base64'));
  assert.equal(await evaluate(`document.documentElement.scrollWidth>innerWidth`),false,'Desktop horizontal overflow');
  await call('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await wait(100);
  assert.equal(await evaluate(`document.documentElement.scrollWidth>innerWidth`),false,'Mobile horizontal overflow');
  assert.deepEqual(errors,[]);
  console.log('PASS: rendering, run/pause/reset, presets, WASD, input focus, key release, Esc, blur, export, responsive layout');
}finally{ws.close();}
