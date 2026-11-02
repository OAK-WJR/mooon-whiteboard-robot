// Exercise the actual firmware page with deliberately reordered/slow replies.
// No connection to hardware and no automatic motion.
const fs=require('fs'),vm=require('vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(process.argv[2]||'c6_drive/web/index.html','utf8').split('<script>')[1].split('</script>')[0];
const elements={},handlers={},calls=[];
const el=id=>elements[id]??={value:'',textContent:'',disabled:false};
const buttons=[...'qweasd'].map(key=>({dataset:{key},classList:{toggle(){}},setPointerCapture(){}}));
const state={session:1,controller:'legacy',armed:false,lease:0,ready:true,reason:'USER_STOP',calibrated:false,
 floor_trial:false,manual_control_available:false,robust_validated:false,hold_validated:false,
 motors:[1,2,3].map(physical=>({physical,target:0,speed:0,pwm:0}))};
const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {promise,resolve};};
let statusGate=null,controllerGate=null,rejectMode=false;
const ctx=vm.createContext({console,Set,Date,Math,Number,JSON,Error,AbortController,AbortSignal,
 localStorage:{getItem:()=>null,setItem(){}},setTimeout:()=>1,clearTimeout(){},
 document:{hidden:false,hasFocus:()=>true,getElementById:el,querySelectorAll:()=>buttons,addEventListener:(k,f)=>handlers[k]=f},
 window:{addEventListener:(k,f)=>handlers[k]=f},
 fetch:async(url,options)=>{
  const command=options?.body?JSON.parse(options.body):null;calls.push({url,command});
  const snapshot=structuredClone(state);const gate=url==='/status'?statusGate:command?.op==='controller'?controllerGate:null;
  return {ok:true,json:async()=>{
   if(gate)await gate.promise;
   if(url==='/status')return snapshot;
   if(command.op==='controller'){
    if(rejectMode)return {ok:false,error:'TEST_REJECTION'};
    state.controller=command.mode;state.floor_trial=command.mode==='floor_trial';
    state.manual_control_available=state.floor_trial;
    state.max_body_speed_mps=state.floor_trial?.08:.4;
    state.max_yaw_rps=state.floor_trial?.4:1.2;
    return {ok:true,controller:state.controller};
   }
   return {ok:true,lease:42};
  }};
 }});
vm.runInContext(source,ctx);
const flush=()=>new Promise(r=>setImmediate(r));
const run=s=>vm.runInContext(s,ctx);
const modes=()=>calls.filter(c=>c.command?.op==='controller');
async function test(){
 await flush();assert.equal(el('mode').value,'legacy');
 // A status response must not reset a focused native picker before change fires.
 el('mode').onfocus?.();el('mode').value='floor_trial';
 await run('tick()');assert.equal(el('mode').value,'floor_trial','Polling overwrote the open picker');
 assert.equal(modes().length,0,'Highlighting does not submit a mode or arm');
 // Reproduce the reported race: a poll is already waiting when selection commits.
 statusGate=deferred();const poll=run('tick()');await flush();
 el('mode').value='floor_trial';await el('mode').onchange();
 assert.equal(el('mode').value,'floor_trial');assert.equal(modes().length,0);
 assert(el('arm').disabled);assert.equal(run('pendingMode'),'floor_trial');
 statusGate.resolve();statusGate=null;await poll;
 assert.equal(modes().length,1);assert.equal(modes()[0].command.mode,'floor_trial');
 assert.equal(el('mode').value,'floor_trial');assert.equal(state.controller,'floor_trial');
 assert(el('arm').disabled,'Do not arm until fresh status confirms mode and limits');
 await run('tick()');assert(!el('arm').disabled);assert.equal(el('speed').max,'0.08');
 el('mode').onblur();await run('tick()');assert.equal(el('mode').value,'floor_trial');
 // A second committed selection while the first is in flight is not lost.
 controllerGate=deferred();el('mode').value='legacy';const first=el('mode').onchange();await flush();
 el('mode').value='floor_trial';await el('mode').onchange();
 controllerGate.resolve();controllerGate=null;await first;
 assert.deepEqual(modes().slice(-2).map(c=>c.command.mode),['legacy','floor_trial']);
 await run('tick()');assert.equal(el('mode').value,'floor_trial');
 // STOP cancels a queued (not yet transmitted) change and never arms.
 statusGate=deferred();const pendingPoll=run('tick()');await flush();
 el('mode').value='legacy';await el('mode').onchange();const count=modes().length;
 await el('stop').onclick();statusGate.resolve();statusGate=null;await pendingPoll;
 assert.equal(modes().length,count);assert.equal(run('pendingMode'),null);
 // STOP during an in-flight change must not display a successful-enable prompt.
 controllerGate=deferred();el('mode').value='floor_trial';const change=el('mode').onchange();await flush();
 await el('stop').onclick();controllerGate.resolve();controllerGate=null;await change;
 assert.equal(el('message').textContent,'Locked');assert.equal(run('lease'),0);
 // A rejected switch reports the error, restores the confirmed value, no retries.
 await run('tick()');rejectMode=true;el('mode').value='legacy';await el('mode').onchange();
 assert(el('message').textContent.includes('TEST_REJECTION'));assert.equal(el('mode').value,'floor_trial');
 const rejectedCount=modes().length;await run('tick()');assert.equal(modes().length,rejectedCount);rejectMode=false;
 // An armed status revokes queued changes; no controller command is emitted.
 state.armed=true;statusGate=deferred();const armedPoll=run('tick()');await flush();
 el('mode').value='legacy';await el('mode').onchange();const armedCount=modes().length;
 statusGate.resolve();statusGate=null;await armedPoll;
 assert.equal(modes().length,armedCount);assert(el('mode').disabled);assert.equal(run('pendingMode'),null);
 assert(!calls.some(c=>['arm','wheels','drive','pwm'].includes(c.command?.op)),'Choosing a mode must never start motors');
 // Explicit enable clicks wait for the active poll, instead of disappearing.
 state.armed=false;await run('tick()');
 statusGate=deferred();const armPoll=run('tick()');await flush();
 await el('arm').onclick();assert.equal(run('armPending'),true);
 assert(!calls.some(c=>c.command?.op==='arm'));
 statusGate.resolve();statusGate=null;await armPoll;
 assert.equal(calls.filter(c=>c.command?.op==='arm').length,1);assert.equal(run('lease'),42);
 await el('stop').onclick();await run('tick()');
 statusGate=deferred();const cancelledArmPoll=run('tick()');await flush();
 await el('arm').onclick();assert.equal(run('armPending'),true);
 await handlers.blur();statusGate.resolve();statusGate=null;await cancelledArmPoll;
 assert.equal(calls.filter(c=>c.command?.op==='arm').length,1,'Blur cancels queued enable');
 assert.equal(run('lease'),0);
 console.log('PASS: focused picker, stale poll, queued change, rapid reselection, STOP, rejection and armed-state races');
}
test().catch(e=>{console.error(e);process.exitCode=1;});
