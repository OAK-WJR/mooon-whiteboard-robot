// Capture only our local simulator in a dedicated, empty headless Chrome profile.
// Usage: node .../capture_portfolio.mjs [--stills]
import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import crypto from 'node:crypto';
import {spawn,execFileSync} from 'node:child_process';
import {once} from 'node:events';
const stills=process.argv.includes('--stills');
const out=new URL('../portfolio/',import.meta.url);fs.mkdirSync(out,{recursive:true});
const pages=await(await fetch('http://127.0.0.1:9238/json/list')).json();
const page=pages.find(p=>p.type==='page');if(!page)throw Error('Dedicated browser missing');
const ws=new WebSocket(page.webSocketDebuggerUrl),pending=new Map(),errors=[];let id=0;
await new Promise((resolve,reject)=>{ws.addEventListener('open',resolve);ws.addEventListener('error',reject);});
ws.addEventListener('message',e=>{const m=JSON.parse(e.data);if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails);
  if(m.id){const p=pending.get(m.id);if(p){clearTimeout(p.timer);pending.delete(m.id);m.error?p.reject(Error(JSON.stringify(m.error))):p.resolve(m.result);}}});
function call(method,params={}){return new Promise((resolve,reject)=>{const n=++id;pending.set(n,{resolve,reject,timer:setTimeout(()=>reject(Error('CDP timeout '+method)),15000)});ws.send(JSON.stringify({id:n,method,params}));});}
async function evaluate(expression){const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
const wait=ms=>new Promise(r=>setTimeout(r,ms));
async function shot(scene,t){await evaluate(`window.portfolio.renderAt('${scene}',${t})`);const r=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});return Buffer.from(r.data,'base64');}
const scenes=[['load','01-load-compensation'],['lateral','02-lateral-coordination'],['slip','03-traction-limit']];
try{
  await call('Runtime.enable');await call('Page.enable');
  await call('Emulation.setDeviceMetricsOverride',{width:1920,height:1080,deviceScaleFactor:1,mobile:false});
  await call('Page.navigate',{url:'http://127.0.0.1:8088/sim/portfolio.html'});
  for(let n=0;n<100;n++){if(await evaluate('!!window.portfolio?.ready'))break;await wait(100);}
  if(!await evaluate('!!window.portfolio?.ready'))throw Error('Presentation not ready');
  await evaluate(`document.querySelector('nav').style.display='none'`);
  const meta=await evaluate('window.portfolio.metadata()');
  const ffmpeg=stills?null:execFileSync('.venv/bin/python',['-c','import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())'],{encoding:'utf8'}).trim();
  for(const [scene,name] of scenes){
    fs.writeFileSync(new URL(name+'.png',out),await shot(scene,10));
    if(stills)continue;
    const destination=new URL(name+'.mp4',out).pathname;
    const proc=spawn(ffmpeg,['-hide_banner','-loglevel','error','-y','-f','image2pipe','-framerate','20','-i','pipe:0','-an','-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',destination],{stdio:['pipe','inherit','inherit']});
    const done=once(proc,'close');let previous=-1,frame;
    for(let i=0;i<280;i++){
      const t=Math.max(0,Math.min(10,(i-20)/20));
      if(t!==previous){frame=await shot(scene,t);previous=t;}
      if(!proc.stdin.write(frame))await once(proc.stdin,'drain');
    }
    proc.stdin.end();const [code]=await done;if(code!==0)throw Error('Video encoding failed');
    console.log('Captured',destination);
  }
  if(errors.length)throw Error(JSON.stringify(errors));
  const hash=f=>crypto.createHash('sha256').update(fs.readFileSync(new URL(f,import.meta.url))).digest('hex');
  fs.writeFileSync(new URL('capture-manifest.json',out),JSON.stringify({created_utc:new Date().toISOString(),language:'English',resolution:[1920,1080],fps:20,
    method:'Deterministic browser-rendered simulation capture; silent; not physical robot footage.',seconds_per_clip:14,core_sha256:hash('core.mjs'),presentation_sha256:hash('portfolio.mjs'),...meta},null,2)+'\n');
}finally{ws.close();}
