// Exercise actual page JavaScript with fake transport; never contacts hardware.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync('c6_drive/web/index.html','utf8');
const source=html.split('<script>')[1].split('</script>')[0];
const handlers={},elements={},buttons=[];
for(const k of 'qweasd')buttons.push({dataset:{key:k},classList:{toggle(){}},setPointerCapture(){}});
const el=id=>elements[id]??=( {value:id==='speed'?'0.08':'',textContent:'',disabled:false} );
const state={firmware:'wbb-c6-drive-1',session:1,armed:false,lease:0,reason:'BOOT',ready:true,calibrated:false,
 motors:[1,2,3].map(physical=>({physical,target:0,speed:0,pwm:0}))};
const calls=[];
const stored=new Map();
const storage={getItem:k=>stored.get(k)??null,setItem:(k,v)=>stored.set(k,String(v))};
let delayedArm=null;
const ctx=vm.createContext({console,Set,Date,Math,Number,JSON,Error,AbortController,AbortSignal,localStorage:storage,
 setTimeout:()=>1,clearTimeout(){},document:{hidden:false,hasFocus:()=>true,getElementById:el,
 querySelectorAll:()=>buttons,addEventListener:(k,v)=>handlers[k]=v},
 window:{addEventListener:(k,v)=>handlers[k]=v},
 fetch:async(url,opt)=>{calls.push([url,opt]);return {ok:true,json:async()=>{
  if(url==='/cmd'&&JSON.parse(opt.body).op==='arm'&&delayedArm)await delayedArm;
  return url==='/status'?state:{ok:true,lease:42};
 }};}});
vm.runInContext(source,ctx);
const primaryNetworkSubmit=el('networkForm').onsubmit,primaryNetworkForget=el('wifiForget').onclick;
async function test(){
 await new Promise(resolve=>setImmediate(resolve));
 assert(el('arm').disabled,'Uncalibrated board must remain locked');
 vm.runInContext('lease=42;key("w",true)',ctx);
 let w=vm.runInContext('wheels()',ctx);assert(w[0]<0&&w[1]===0&&w[2]>0);
 handlers.keyup({key:'w'});assert(vm.runInContext('keys.size',ctx)===0);
 vm.runInContext('key("a",true);key("w",true)',ctx);
 w=vm.runInContext('wheels()',ctx);assert(w.every(x=>Number.isFinite(x)&&Math.abs(x)<8000));
 buttons[0].onpointerdown({preventDefault(){},pointerId:1});buttons[0].onpointercancel();
 assert(!vm.runInContext('keys.has("q")',ctx));
 await handlers.blur();assert(vm.runInContext('lease',ctx)===0);assert(vm.runInContext('keys.size',ctx)===0);
 assert(calls.some(([url,opt])=>url==='/cmd'&&JSON.parse(opt.body).op==='stop'));
 assert.equal(vm.runInContext('driveSpeed',ctx),.18,'New default speed');
 el('maxSpeed').value='.40';el('maxSpeed').onchange();el('useMax').onclick();
 assert.equal(vm.runInContext('driveSpeed',ctx),.40);
 assert.equal(el('speed').max,'0.40');
 el('maxSpeed').value='.12';el('maxSpeed').onchange();
 assert.equal(vm.runInContext('driveSpeed',ctx),.12,'Lowering maximum clamps current speed');
 el('maxSpeed').value='9';el('maxSpeed').onchange();el('useMax').onclick();
 assert.equal(vm.runInContext('driveSpeed',ctx),.40,'Manual maximum is bounded');
 assert.equal(JSON.parse(stored.get('wbb-drive-speeds-v1')).maximum,.40);
 el('maxSpeed').value='';el('maxSpeed').onchange();assert.equal(el('maxSpeed').value,'0.40');
 el('speed').value='NaN';el('speed').oninput();assert.equal(vm.runInContext('driveSpeed',ctx),.40);
 vm.runInContext('lease=42;key("w",true)',ctx);el('maxSpeed').onfocus();
 handlers.keydown({key:'e',target:{tagName:'INPUT'},preventDefault(){throw Error('Input was hijacked');}});
 assert.equal(vm.runInContext('keys.size',ctx),0,'Editing speed must not drive');
 // Every combination of translation/rotation keys stays within either firmware cap.
 for(const limit of [8000,10000])for(let mask=0;mask<64;mask++){
  vm.runInContext(`maxWheelCps=${limit};keys.clear();`,ctx);
  [...'wasdqe'].forEach((k,i)=>{if(mask&(1<<i))vm.runInContext(`keys.add('${k}')`,ctx);});
  w=vm.runInContext('wheels()',ctx);
  assert(w.every(x=>Number.isFinite(x)&&Math.abs(x)<=limit+1e-8));
 }
 vm.runInContext('keys.clear();key("a",true);key("q",true);maxWheelCps=10000',ctx);
 w=vm.runInContext('wheels()',ctx);
 const raw=[.20+.033738*1.2,-.40+.033738*1.2,.20+.033738*1.2];
 assert(Math.abs(w[0]/w[1]-raw[0]/raw[1])<1e-9,'Saturation preserves wheel ratios');
 await handlers.blur();
 state.calibrated=true;await vm.runInContext('tick()',ctx);
 let releaseArm;delayedArm=new Promise(resolve=>{releaseArm=resolve;});
 const arming=el('arm').onclick();await Promise.resolve();
 await handlers.blur();releaseArm();await arming;
 assert(vm.runInContext('lease',ctx)===0,'A late arm response must not undo a stop');
 // Reload the actual script with persisted settings, including a lower saved cap.
 stored.set('wbb-drive-speeds-v1',JSON.stringify({maximum:.15,current:.40}));
 const primaryHandlers={...handlers};
 const reloaded=vm.createContext({...ctx});vm.runInContext(source,reloaded);
 await new Promise(resolve=>setImmediate(resolve));
 assert.equal(vm.runInContext('driveSpeed',reloaded),.15);
 assert.equal(vm.runInContext('lease',reloaded),0,'Reload must stay disarmed');
 // The real UI sends body velocities in hold mode and zero heartbeats on release.
 state.calibrated=true;state.robust_validated=true;state.hold_validated=true;
 state.controller='encoder_hold';state.armed=true;state.lease=42;
 state.odometry={valid:true,x_m:.012,y_m:-.034,yaw_rad:.1,holding:true};
 vm.runInContext('lease=42;controlMode="encoder_hold";keys.clear();keys.add("w");lastStatus=Date.now()',ctx);
 await vm.runInContext('tick()',ctx);
 let sent=calls.filter(([url])=>url==='/cmd').map(([,o])=>JSON.parse(o.body));
 assert(sent.some(d=>d.op==='drive'&&d.values[0]===.08&&d.values[1]===0));
 assert.equal(el('speedValue').textContent,'0.08');
 assert.equal(el('speed').max,'0.08');
 assert(el('mode').disabled);assert(el('pose').textContent.includes('12.0 mm'));
 primaryHandlers.keyup({key:'w'});await vm.runInContext('tick()',ctx);
 sent=calls.filter(([url])=>url==='/cmd').map(([,o])=>JSON.parse(o.body));
 const lastDrive=sent.filter(d=>d.op==='drive').at(-1);assert.deepEqual(lastDrive.values,[0,0,0]);
 await primaryHandlers.blur();assert.equal(vm.runInContext('lease',ctx),0);
 // A fresh page must show latched fault evidence even after USER_STOP overwrites
 // the live state reason. It must never call this a current fault after re-arming.
 state.armed=false;state.lease=0;state.reason='USER_STOP';state.control_run=3;
 state.last_fault={code:'START_NO_MOTION',motor:2,ms:12340,run:3,mode:'encoder_hold',dt_s:.01,
  target_cps:[500,600,700],speed_cps:[490,0,695],applied_pwm:[80,140,90]};
 const faultStart=calls.length;await vm.runInContext('tick()',ctx);
 assert.equal(el('faultPanel').hidden,false);
 assert(el('faultSummary').textContent.includes('Motor 2'));
 assert(el('faultSummary').textContent.includes('Startup compensation ended'));
 assert(el('faultData').textContent.includes('PWM 140/255'));
 assert(!calls.slice(faultStart).some(([u,o])=>u==='/cmd'),'Fault display must not send commands');
 state.armed=true;state.control_run=4;await vm.runInContext('tick()',ctx);
 assert(el('faultSummary').textContent.includes('History'));
 state.armed=false;delete state.last_fault;state.reason='ROBUST_INPUT_FAULT';
 await vm.runInContext('tick()',ctx);
 assert.equal(el('faultPanel').hidden,true);
 assert(el('message').textContent.includes('cannot identify the faulty motor'));
 assert(el('modeLimitNote').textContent.includes('not for wall hold'));
 // Provisioning uses the existing page/token, never browser password storage,
 // never an automatic arm, and is locked while any controller is active.
 state.network={configured:false,connected:false,busy:false,state:'ap_only',ssid:'',
  station_url:'',mdns_url:'http://wbb-test.local/',ap_url:'http://192.168.4.1/'};
 await vm.runInContext('tick()',ctx);
 assert(!el('wifiSave').disabled);assert(el('homeUrl').hidden);
 state.network={...state.network,configured:true,state:'retry_wait',disconnect_reason:201,disconnect_name:'NO_AP_FOUND'};
 await vm.runInContext('tick()',ctx);
 assert(el('networkState').textContent.includes('2.4 GHz'));
 state.network.disconnect_reason=23;await vm.runInContext('tick()',ctx);
 assert(el('networkState').textContent.includes('school access policy'),'Authentication failure is not proof of a wrong password');
 state.network.associated=true;state.network.state='connecting';await vm.runInContext('tick()',ctx);
 assert(el('networkState').textContent.includes('waiting for IP'));
 state.network.associated=false;state.network.state='retry_wait';
 state.network.scan_state='complete';state.network.scan_ap_count=12;state.network.scan_matches=0;
 await vm.runInContext('tick()',ctx);
 assert(el('networkState').textContent.includes('0 match this network'));
 el('wifiSsid').value='Test school';el('wifiPassword').value='test-only-password';
 const beforeNetwork=calls.length;
 await primaryNetworkSubmit({preventDefault(){}});
 const posted=calls.slice(beforeNetwork).filter(([url])=>url==='/network');
 assert.equal(posted.length,1);assert.equal(JSON.parse(posted[0][1].body).ssid,'Test school');
 assert.equal(posted[0][1].headers['X-Control-Token'],'__TOKEN__');
 assert.equal(el('wifiPassword').value,'');
 assert(!JSON.stringify([...stored]).includes('test-only-password'));
 assert(!calls.slice(beforeNetwork).some(([u,o])=>u==='/cmd'&&JSON.parse(o.body).op==='arm'));
 state.network={...state.network,connected:true,state:'connected',station_url:'http://10.0.0.77/',ssid:'<img src=x onerror=alert(1)>'};
 await vm.runInContext('tick()',ctx);
 assert.equal(el('homeUrl').href,'http://10.0.0.77/');assert(!el('homeUrl').hidden);
 assert(!el('networkState').textContent.includes('Last disconnect'),'Old failure must not obscure a successful connection');
 assert(el('networkState').textContent.includes('<img'));
 vm.runInContext('networkLink("homeUrl","javascript:alert(1)","test")',ctx);assert(el('homeUrl').hidden);
 state.armed=true;await vm.runInContext('tick()',ctx);assert(el('wifiSave').disabled);
 const activeCalls=calls.length;await primaryNetworkForget();
 assert.equal(calls.length,activeCalls,'Cannot change network while armed');
 state.armed=false;state.network.busy=true;await vm.runInContext('tick()',ctx);
 assert(el('arm').disabled);assert(el('wifiSave').disabled);
 state.network.busy=false;await vm.runInContext('tick()',ctx);
 el('wifiAuth').value='peap';vm.runInContext('enterpriseFields()',ctx);
 assert.equal(el('enterpriseFields').hidden,false);assert(el('wifiCA').required);
 el('wifiUsername').value='test-account';el('wifiPassword').value='test-only-account-password';
 const beforeIncomplete=calls.length;await primaryNetworkSubmit({preventDefault(){}});
 assert.equal(calls.length,beforeIncomplete,'Missing CA/domain must not submit account credentials');
 el('wifiServer').value='radius.school.example';
 el('wifiCA').value='-----BEGIN CERTIFICATE-----\ntest-only-format-fixture\n-----END CERTIFICATE-----';
 el('wifiIdentity').value='anonymous@school.example';
 await primaryNetworkSubmit({preventDefault(){}});
 const enterprise=JSON.parse(calls.filter(([u])=>u==='/network').at(-1)[1].body);
 assert.equal(enterprise.auth,'peap');assert.equal(enterprise.username,'test-account');
 assert.equal(enterprise.server_name,'radius.school.example');assert(enterprise.ca_pem.includes('CERTIFICATE'));
 assert.equal(el('wifiPassword').value,'');
 assert(!JSON.stringify([...stored]).includes('test-account'));
 for(const method of ['ttls_mschapv2','ttls_pap']){
  state.network.busy=false;await vm.runInContext('tick()',ctx);el('wifiAuth').value=method;
  el('wifiPassword').value='test-only-account-password';await primaryNetworkSubmit({preventDefault(){}});
  assert.equal(JSON.parse(calls.filter(([u])=>u==='/network').at(-1)[1].body).auth,method);
 }
 state.network.busy=false;await vm.runInContext('tick()',ctx);
 state.armed=false;await vm.runInContext('tick()',ctx);
 // Connected saved Wi-Fi is informational; empty password must never erase it.
 state.network={...state.network,configured:true,connected:true,auth:'personal',ssid:'Saved Test Network',password_saved:true,configure_over_lan:true};
 el('wifiSsid').value='Saved Test Network';el('wifiAuth').value='personal';el('wifiPassword').value='';
 await vm.runInContext('tick()',ctx);
 assert(el('networkHint').textContent.includes('no need to save again'));
 assert(el('wifiPassword').placeholder.includes('leave blank to keep it'));
 const unchangedPosts=calls.filter(([u])=>u==='/network').length;
 await primaryNetworkSubmit({preventDefault(){}});
 assert.equal(calls.filter(([u])=>u==='/network').length,unchangedPosts,'Connected unchanged form does not reconnect');
 assert.equal(el('networkSettings').open,false);
 state.network.connected=false;state.network.state='retry_wait';await vm.runInContext('tick()',ctx);
 await primaryNetworkSubmit({preventDefault(){}});
 const reuse=JSON.parse(calls.filter(([u])=>u==='/network').at(-1)[1].body);
 assert.equal(reuse.reuse_saved_password,true);assert.equal(reuse.password,'');
 state.network.busy=false;state.network.connected=true;await vm.runInContext('tick()',ctx);
 // Polling cannot replace a partly typed speed limit; password spaces are text.
 ctx.document.activeElement=el('maxSpeed');el('maxSpeed').value='0.';await vm.runInContext('tick()',ctx);
 assert.equal(el('maxSpeed').value,'0.');ctx.document.activeElement=null;
 const spaceCalls=calls.length;
 primaryHandlers.keydown({key:' ',target:{tagName:'INPUT',type:'password'},preventDefault(){throw Error('Password space was hijacked');}});
 assert.equal(calls.length,spaceCalls);
 // Limited manual trials enable the original UI without falsifying calibration.
 Object.assign(state,{controller:'floor_trial',floor_trial:true,manual_control_available:true,
  calibrated:false,robust_validated:false,hold_validated:false,max_body_speed_mps:.08,max_yaw_rps:.4});
 vm.runInContext('lease=0;keys.clear()',ctx);
 const beforeTrial=calls.length;await vm.runInContext('tick()',ctx);
 assert(!el('arm').disabled);assert(!el('floorOption').disabled);
 assert(el('robustOption').disabled&&el('holdOption').disabled);
 assert.equal(el('speed').max,'0.08');assert.equal(el('maxSpeed').max,'0.08');
 assert(!calls.slice(beforeTrial).some(([u,o])=>u==='/cmd'&&JSON.parse(o.body).op==='arm'));
 for(let mask=0;mask<64;mask++){
  vm.runInContext('keys.clear();driveSpeed=.4',ctx);
  [...'wasdqe'].forEach((k,i)=>{if(mask&(1<<i))vm.runInContext(`keys.add('${k}')`,ctx);});
  const [x,y,yaw]=vm.runInContext('bodyCommand()',ctx);
  assert(Math.hypot(x,y)<=.08000001&&Math.abs(yaw)<=.40000001);
 }
 Object.assign(state,{armed:true,lease:42});
 vm.runInContext('lease=42;keys.clear();lastStatus=Date.now()',ctx);
 await vm.runInContext('tick()',ctx);
 const release=calls.filter(([u])=>u==='/cmd').map(([,o])=>JSON.parse(o.body)).filter(d=>d.op==='wheels').at(-1);
 assert.deepEqual(release.values,[0,0,0],'Normal release sends zero speed for coordinated deceleration');
 await primaryHandlers.blur();assert.equal(vm.runInContext('lease',ctx),0);
 Object.assign(state,{armed:false,lease:0,controller:'legacy',floor_trial:false,manual_control_available:false});
 await vm.runInContext('tick()',ctx);assert(el('arm').disabled,'A trial does not qualify other modes');
 // The compiled page and tested source must remain identical.
 const header=fs.readFileSync('c6_drive/include/web_ui.h','utf8');
 assert.equal(header.split('R"WBBPAGE(')[1].split(')WBBPAGE";')[0],html);
 console.log('Speed editing/storage, all key combinations, stop handling and calibration lock: PASS');
}test().catch(e=>{console.error(e);process.exitCode=1});
