// Render the actual firmware UI in a dedicated local browser, mocked transport.
// Never connects to the robot or enumerates personal browser tabs.
import fs from 'node:fs';
import assert from 'node:assert/strict';
const pages=await(await fetch('http://127.0.0.1:9238/json/list')).json();
const socket=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
await new Promise((r,j)=>{socket.addEventListener('open',r);socket.addEventListener('error',j);});
let id=0;const pending=new Map();
socket.addEventListener('message',e=>{const m=JSON.parse(e.data);if(m.id&&pending.has(m.id)){const p=pending.get(m.id);pending.delete(m.id);clearTimeout(p.timer);m.error?p.reject(Error(JSON.stringify(m.error))):p.resolve(m.result);}});
function call(method,params={}){return new Promise((resolve,reject)=>{const n=++id;pending.set(n,{resolve,reject,timer:setTimeout(()=>reject(Error(method+' timeout')),10000)});socket.send(JSON.stringify({id:n,method,params}));});}
async function js(expression){const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
try{
 await call('Page.enable');await call('Page.navigate',{url:'about:blank'});
 await new Promise(r=>setTimeout(r,150));
 await js(`window.testRequests=[];window.fetch=async(url,options)=>{window.testRequests.push({url,options});return {ok:true,json:async()=>url==='/status'?{controller:'encoder_hold',session:1,ready:true,calibrated:true,robust_validated:true,hold_validated:true,armed:false,lease:0,reason:'USER_STOP',max_wheel_cps:10000,network:{configured:true,connected:true,busy:false,state:'connected',ssid:'Test Network',station_url:'http://10.0.0.77/',mdns_url:'http://wbb-c6-test.local/',ap_url:'http://192.168.4.1/',rssi_dbm:-48},odometry:{valid:true,x_m:.023,y_m:-.012,yaw_rad:.02,holding:false},motors:[1,2,3].map(physical=>({physical,target:0,speed:0,pwm:0}))}:{ok:true}}}`);
 await call('Page.setDocumentContent',{frameId:(await call('Page.getFrameTree')).frameTree.frame.id,html:fs.readFileSync('c6_drive/web/index.html','utf8')});
 await new Promise(r=>setTimeout(r,200));
 assert((await js(`document.getElementById('pose').textContent`)).includes('23.0 mm'));
 assert.equal(await js(`document.getElementById('homeUrl').href`),'http://10.0.0.77/');
 assert((await js(`document.getElementById('networkState').textContent`)).includes('Test Network'));
 await js(`document.getElementById('wifiSsid').value='New Test Network';document.getElementById('wifiPassword').value='test-only-password';document.getElementById('networkForm').onsubmit({preventDefault(){}})`);
 assert.equal(await js(`document.getElementById('wifiPassword').value`),'');
 assert.equal(await js(`window.testRequests.filter(r=>r.url==='/network').length`),1);
 assert.equal(await js(`window.testRequests.some(r=>r.url==='/cmd'&&JSON.parse(r.options.body).op==='arm')`),false);
 for(const [name,width,height,mobile] of [['desktop',1180,1080,false],['mobile',390,844,true]]){
  await call('Emulation.setDeviceMetricsOverride',{width,height,deviceScaleFactor:1,mobile});
  assert.equal(await js(`document.documentElement.scrollWidth>innerWidth`),false,'Horizontal overflow: '+name);
  const img=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
  fs.writeFileSync(`c6_drive/reports/encoder-hold-ui-${name}.png`,Buffer.from(img.data,'base64'));
  await js(`document.getElementById('networkPanel').scrollIntoView({block:'start'})`);
  const networkImg=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
  fs.writeFileSync(`c6_drive/reports/network-ui-${name}.png`,Buffer.from(networkImg.data,'base64'));
  await js(`document.getElementById('wifiAuth').value='peap';enterpriseFields();document.getElementById('enterpriseFields').scrollIntoView({block:'start'})`);
  assert.equal(await js(`document.getElementById('enterpriseFields').hidden`),false);
  assert.equal(await js(`document.documentElement.scrollWidth>innerWidth`),false,'Enterprise form overflow: '+name);
  const enterpriseImg=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
  fs.writeFileSync(`c6_drive/reports/network-enterprise-${name}.png`,Buffer.from(enterpriseImg.data,'base64'));
  await js(`document.getElementById('wifiAuth').value='personal';enterpriseFields()`);
 }
 console.log('PASS: actual firmware UI, network provisioning, no automatic arm, mock state and desktop/mobile layout');
}finally{socket.close();}
