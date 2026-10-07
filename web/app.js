import {decode,euler,rotate} from './protocol.js';
const $=id=>document.getElementById(id);
const UUID={service:'6e400001-b5a3-f393-e0a9-e50e24dcca9e',rx:'6e400002-b5a3-f393-e0a9-e50e24dcca9e',tx:'6e400003-b5a3-f393-e0a9-e50e24dcca9e'};
const colors=['#ff7878','#53e1b3','#6aa8ff'];
let device=null,rx=null,tx=null,bat=null,ready=false,connecting=false,demo=false;
let session=0,chain=Promise.resolve(),pending=0,batteryTimer=null,latest=null,history=[],lastPacket=0,counter=0;
const faces=[];
function box(x,y,z,w,h,t,color) {
  const v=Array.from({length:8},(_,i)=>[x+((i&1)?w:-w)/2,y+((i&2)?h:-h)/2,z+((i&4)?t:-t)/2]);
  for(const f of [[0,2,3,1],[4,5,7,6],[0,1,5,4],[2,6,7,3],[0,4,6,2],[1,3,7,5]]) faces.push({v:f.map(i=>v[i]),color});
}
box(0,0,0,1.55,2.05,.10,'#1c8461');
box(0,-.72,.13,.70,.55,.28,'#b8c8d0');
box(0,.15,.14,1.02,.93,.22,'#435561');
box(0,.82,.08,.84,.22,.07,'#1e2632');
for(const s of [-1,1]) for(let i=0;i<7;i++) box(s*.72,-.82+i*.27,.06,.19,.13,.06,'#e0b95b');
function controls() {
  $('connect').textContent=device?'Disconnect':connecting?'Connecting…':'Connect board';
  $('connect').disabled=connecting;
  for(const id of ['zero','calibrate','rate']) $(id).disabled=!ready;
  $('demo').disabled=!!device||connecting;
  $('demo').textContent=demo?'Stop demo':'Preview demo';
  $('badge').textContent=demo?'DEMO':ready?'LIVE':'OFFLINE';
  $('badge').className='badge'+(demo?' demo':ready?' live':'');
}
function reset(message='Disconnected. Tap Connect board to retry.') {
  ++session;ready=false;connecting=false;
  if(tx) tx.removeEventListener('characteristicvaluechanged',notification);
  if(device) {device.removeEventListener('gattserverdisconnected',disconnected);if(device.gatt.connected) device.gatt.disconnect();}
  device=rx=tx=bat=null;
  clearInterval(batteryTimer);batteryTimer=null;
  chain=Promise.resolve();pending=0;
  $('battery').textContent='Battery —';$('status').textContent=message;controls();
}
function disconnected() {reset();}
async function bounded(promise) {
  let timer;
  try {return await Promise.race([promise,new Promise((_,reject)=>{timer=setTimeout(()=>reject(new Error('Bluetooth operation timed out')),10000);})]);}
  finally {clearTimeout(timer);}
}
function queue(fn) {
  const token=session;
  if(pending>=8) return Promise.resolve();
  ++pending;
  const run=chain.then(async()=>{if(token!==session||!device) return;return bounded(fn());});
  chain=run.catch(e=>{if(token===session) reset(e.message+'. Reconnect to retry.');}).finally(()=>{if(token===session) --pending;});
  return run;
}
function command(c) {
  if(!ready) return;
  return queue(()=>rx.writeValueWithResponse(new Uint8Array([c.charCodeAt(0)])));
}
function notification(event) {
  if(!ready||event.target!==tx) return;
  const p=decode(event.target.value);if(p) accept(p);
}
function accept(p) {
  latest=p;lastPacket=performance.now();counter++;
  history.push({...p,t:lastPacket});
  history=history.filter(p=>lastPacket-p.t<=5000).slice(-500);
  for(let i=0;i<3;i++) $(['roll','pitch','yaw'][i]).textContent=p.rpy[i].toFixed(1)+'°';
}
$('connect').addEventListener('click',async()=>{
  if(device) {reset();return;}
  if(!navigator.bluetooth) {$('status').textContent='Web Bluetooth is unavailable here. Use a supported Chrome/Edge browser or the Android app.';return;}
  demo=false;latest=null;history=[];lastPacket=0;counter=0;
  connecting=true;controls();
  const token=++session;
  let selected;
  try {
    // Must stay directly inside the click handler, before any awaited work.
    selected=await navigator.bluetooth.requestDevice({filters:[{services:[UUID.service]},{name:'XIAO-IMU'}],optionalServices:[UUID.service,'battery_service']});
    if(token!==session) return;
    device=selected;device.addEventListener('gattserverdisconnected',disconnected);
    $('status').textContent='Connecting to '+(device.name||'Nordic UART board')+'…';
    const server=await bounded(device.gatt.connect());
    if(token!==session) {if(selected.gatt.connected) selected.gatt.disconnect();return;}
    const service=await bounded(server.getPrimaryService(UUID.service));
    const newRx=await bounded(service.getCharacteristic(UUID.rx));
    const newTx=await bounded(service.getCharacteristic(UUID.tx));
    if(token!==session) return;
    rx=newRx;tx=newTx;tx.addEventListener('characteristicvaluechanged',notification);
    await bounded(tx.startNotifications());
    if(token!==session) return;
    await bounded(rx.writeValueWithResponse(new Uint8Array([98]))); // b, 50Hz
    if(token!==session) return;
    ready=true;connecting=false;$('rate').value='2';controls();
    $('status').textContent='Connected. Keep still until boot calibration finishes.';
    try {
      const bs=await bounded(server.getPrimaryService('battery_service'));
      const level=await bounded(bs.getCharacteristic('battery_level'));
      if(token!==session) return;
      bat=level;
      await queue(async()=>{$('battery').textContent='Battery ~'+(await bat.readValue()).getUint8(0)+'%';});
      if(token!==session) return;
      batteryTimer=setInterval(()=>{if(ready&&pending===0) queue(async()=>{$('battery').textContent='Battery ~'+(await bat.readValue()).getUint8(0)+'%';});},10000);
    } catch(e) {if(token===session&&ready) $('battery').textContent='Battery unavailable';}
  } catch(e) {if(token===session) reset(e.name==='NotFoundError'?'Selection cancelled.':e.message);}
});
$('zero').onclick=()=>command('z');
$('calibrate').onclick=()=>{if(confirm('Set the board down and keep it still for two seconds. Start gyro calibration?')) {command('c');$('status').textContent='Calibrating • keep the board still';}};
$('rate').onchange=()=>command($('rate').value);
$('demo').onclick=()=>{
  demo=!demo;history=[];latest=null;lastPacket=0;counter=0;
  $('status').textContent=demo?'Demo preview • simulated motion, no board connected.':'Connect your XIAO, then keep it still for two seconds.';
  for(const id of ['roll','pitch','yaw']) $(id).textContent='—';controls();
};
window.addEventListener('pagehide',()=>reset('Connection closed'));
setInterval(()=>{
  if(!ready) return;
  const age=performance.now()-lastPacket;
  $('status').textContent=lastPacket===0||age>3000?'No motion yet • keep still to calibrate; check the red fault LED.':
    age>800?'Motion paused • calibration or connection stalled.':'Live motion • '+counter+' samples / second';
  counter=0;
},1000);
function surface(id) {
  const canvas=$(id),r=canvas.getBoundingClientRect(),dpr=Math.min(window.devicePixelRatio||1,2);
  if(canvas.width!==Math.round(r.width*dpr)||canvas.height!==Math.round(r.height*dpr)) {canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);}
  const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,r.width,r.height);
  return {c,w:r.width,h:r.height};
}
function camera(v,q) {const [a,b,d]=rotate(v,q);return [a,.88*b-.475*d,.475*b+.88*d];}
function board() {
  const {c,w,h}=surface('board'),q=latest?.q||[1,0,0,0],scale=Math.min(w*.21,h*.31);
  const project=v=>{const k=5/(5-v[2]);return [w/2+v[0]*scale*k,h*.52-v[1]*scale*k];};
  const sorted=faces.map(f=>{const v=f.v.map(v=>camera(v,q));return {p:v.map(project),z:v.reduce((s,p)=>s+p[2],0)/4,color:f.color};}).sort((a,b)=>a.z-b.z);
  for(const f of sorted) {
    c.beginPath();f.p.forEach(([x,y],i)=>i?c.lineTo(x,y):c.moveTo(x,y));c.closePath();
    c.fillStyle=f.color;c.fill();c.strokeStyle='#09161e66';c.lineWidth=.7;c.stroke();
  }
  const origin=project(camera([0,0,.4],q));
  for(let i=0;i<3;i++) {
    const v=[0,0,.4];v[i]+=1.45;const end=project(camera(v,q));
    c.beginPath();c.moveTo(...origin);c.lineTo(...end);c.strokeStyle=colors[i];c.lineWidth=2;c.stroke();
    c.fillStyle=colors[i];c.font='12px system-ui';c.fillText('XYZ'[i],end[0]+5,end[1]);
  }
}
function chart(id,key,minRange) {
  const {c,w,h}=surface(id),now=performance.now();
  const samples=history.filter(p=>now-p.t<=5000);
  let range=minRange;
  for(const p of samples) for(const n of p[key]) range=Math.max(range,Math.abs(n)*1.08);
  c.font='10px system-ui';c.textAlign='right';c.fillStyle='#8fa6bc';c.fillText('±'+range.toFixed(1),w-4,12);
  c.strokeStyle='#2c3d51';c.lineWidth=1;
  for(const y of [h*.25,h*.5,h*.75]) {c.beginPath();c.moveTo(0,y);c.lineTo(w,y);c.stroke();}
  for(let i=0;i<3;i++) {
    c.beginPath();let first=true;
    for(const p of samples) {const x=w*(1-(now-p.t)/5000),y=h/2-p[key][i]/range*(h/2-16);if(first){c.moveTo(x,y);first=false;}else c.lineTo(x,y);}
    c.strokeStyle=colors[i];c.lineWidth=1.7;c.stroke();
  }
}
let demoAt=0;
function frame(time) {
  if(demo&&time-demoAt>=20) {
    demoAt=time;
    const t=time/1800,roll=.4*Math.sin(t),pitch=.3*Math.cos(t*.7),yaw=t*.3;
    const cr=Math.cos(roll/2),sr=Math.sin(roll/2),cp=Math.cos(pitch/2),sp=Math.sin(pitch/2),cy=Math.cos(yaw/2),sy=Math.sin(yaw/2);
    const q=[cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy];
    // Gravity in body frame for the simulated orientation.
    accept({q,a:rotate([0,0,1],[q[0],-q[1],-q[2],-q[3]]),g:[13*Math.cos(t),-7*Math.sin(t*.7),9.55],rpy:euler(q)});
  }
  board();chart('accel','a',1.2);chart('gyro','g',100);requestAnimationFrame(frame);
}
controls();requestAnimationFrame(frame);
