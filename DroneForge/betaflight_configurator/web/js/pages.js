(function(){
const {h,S,sect,note,field,fields}=BF;
const P=BF.pages;
const title=t=>h('h1',{class:'pt'},t);
const live=()=>S.live||{};
BF.liveHook=f=>{BF.liveCbs=[f]};
// ---------- Setup
const quad=(cv,r,p,y)=>{const c=cv.getContext('2d'),W=cv.width,Hh=cv.height;c.fillStyle='#222';c.fillRect(0,0,W,Hh);
 const rot=([x,yy,z])=>{let a=r*Math.PI/180,b=p*Math.PI/180,g=y*Math.PI/180;
  let y1=yy*Math.cos(b)-z*Math.sin(b),z1=yy*Math.sin(b)+z*Math.cos(b);let x2=x*Math.cos(a)+z1*Math.sin(a),z2=-x*Math.sin(a)+z1*Math.cos(a);
  return [x2*Math.cos(g)-y1*Math.sin(g),x2*Math.sin(g)+y1*Math.cos(g),z2]};
 const pr=v=>{const q=rot(v),k=300/(300+q[1]*30+q[2]*0);return [W/2+q[0]*70*k,Hh/2-q[2]*70*k+q[1]*18]};
 const arms=[[-1,1,0],[1,1,0],[1,-1,0],[-1,-1,0]],C=pr([0,0,0]);c.lineWidth=5;c.lineCap='round';
 arms.forEach((a,i)=>{const e=pr(a);c.strokeStyle=i<2?'#ffbb00':'#aaa';c.beginPath();c.moveTo(...C);c.lineTo(...e);c.stroke();
  c.strokeStyle='#5dade2';c.lineWidth=2;c.beginPath();c.ellipse(e[0],e[1],28,10,0,0,7);c.stroke();c.lineWidth=5});
 c.fillStyle='#444';c.fillRect(C[0]-14,C[1]-9,28,18);const f=pr([0,1.6,0]);c.strokeStyle='#e33';c.lineWidth=3;c.beginPath();c.moveTo(...C);c.lineTo(...f);c.stroke();
 c.fillStyle='#ccc';c.font='11px sans-serif';c.fillText('front = red/yellow',8,Hh-8)};
const horizon=(cv,r,p)=>{const c=cv.getContext('2d'),W=cv.width,Hh=cv.height,R=Hh/2-4;c.clearRect(0,0,W,Hh);c.save();c.translate(W/2,Hh/2);
 c.beginPath();c.arc(0,0,R,0,7);c.clip();c.rotate(-r*Math.PI/180);const off=p*3;c.fillStyle='#4a90c8';c.fillRect(-R*2,-R*2,R*4,R*2+off);c.fillStyle='#8a5a2b';c.fillRect(-R*2,off,R*4,R*2);
 c.strokeStyle='#fff';c.lineWidth=2;c.beginPath();c.moveTo(-R,off);c.lineTo(R,off);c.stroke();c.font='10px sans-serif';c.fillStyle='#fff';
 for(let i=-30;i<=30;i+=10){if(!i)continue;c.beginPath();c.moveTo(-18,off+i*3);c.lineTo(18,off+i*3);c.stroke();c.fillText(-i,22,off+i*3+3)}c.restore();
 c.strokeStyle='#ffbb00';c.lineWidth=3;c.beginPath();c.moveTo(W/2-45,Hh/2);c.lineTo(W/2-12,Hh/2);c.lineTo(W/2,Hh/2+10);c.lineTo(W/2+12,Hh/2);c.lineTo(W/2+45,Hh/2);c.stroke();
 c.beginPath();c.arc(W/2,Hh/2,R,0,7);c.stroke()};
P.setup={label:'Setup',icon:'M3 17l9-12 9 12M7 17l5-6 5 6',mode:'live',render(el){
 const a=h('canvas',{width:300,height:300}),q=h('canvas',{width:420,height:300});
 const t=(id,l,u)=>h('div',{class:'tile'},h('b',{id},'-'),h('small',{},l+(u?' ('+u+')':'')));
 const i=S.info||{};
 el.append(title('Setup'),h('div',{class:'cols'},
  sect('Attitude',h('div',{style:'display:flex;gap:10px;flex-wrap:wrap'},a,q),h('div',{id:'att',class:'muted',style:'margin-top:6px'},'roll / pitch / yaw')),
  h('div',{},sect('Accelerometer calibration',note('Place the drone LEVEL on a flat surface and keep it STILL (no vibration, props on is fine) before pressing. Works only while disarmed.'),
    h('button',{class:'orange',onclick:async()=>{if(await BF.try('fc.calibrate_acc'))BF.banner('info','Accelerometer calibration started: keep the drone level and still for 3 seconds.')}},'Calibrate accelerometer')),
   sect('Battery and link',h('div',{class:'tiles'},t('t-v','Battery','V'),t('t-a','Current','A'),t('t-r','RSSI','%'),t('t-cl','Range','m'))),
   sect('Firmware',h('div',{},'Firmware: '+(i.variant||'-')+' '+(i.version||'-')+'  API '+(i.api||'-')),h('div',{},'Board: '+(i.board||'-')),h('div',{},'Build: '+(i.build||'-')),h('div',{},'UID: '+(i.uid||'-')),
    h('div',{},'Armed: '+(i.armed?'YES':'no')),h('div',{style:'margin-top:8px'},h('button',{class:'red',title:'Puts the FC in DFU bootloader for flashing',onclick:()=>BF.try('fc.reboot',{target:'bootloader'})},'Reboot to BOOTLOADER (DFU, for flashing only)'))))));
 const $=id=>el.querySelector('#'+id);
 BF.liveHook(()=>{const l=live();if(!l.attitude)return;const at=l.attitude;horizon(a,at.roll,at.pitch);quad(q,at.roll,at.pitch,at.yaw);
  $('att').textContent=`roll ${at.roll.toFixed(1)}  pitch ${at.pitch.toFixed(1)}  yaw ${at.yaw.toFixed(0)} deg`;const an=l.analog||{};
  $('t-v').textContent=(an.vbat??0).toFixed(2);$('t-a').textContent=(an.amps??0).toFixed(1);$('t-r').textContent=an.rssi??0;$('t-cl').textContent=l.range_cm!=null?(l.range_cm/100).toFixed(2):'-'});
 horizon(a,0,0);quad(q,0,0,0)}};
// ---------- Ports
const FN={1:'MSP',2:'GPS',4:'FRSKY',8:'HOTT',16:'LTM',32:'SMARTPORT',64:'RX_SERIAL',128:'BLACKBOX',512:'MAVLINK',1024:'ESC_SENSOR',2048:'SMARTAUDIO',4096:'TRAMP',8192:'IBUS',262144:'MTF-02P range/flow (MSP rangefinder)'};
const uartName=id=>id==20?'USB VCP':id>=30?'SOFTSERIAL'+(id-29):'UART'+(+id+1);
P.ports={label:'Ports',icon:'M4 8h16v8H4zM8 8V5M12 8V5M16 8V5M8 16v3M16 16v3',mode:'settings',render(el){
 el.append(title('Ports'),note('serial 2 262144 = UART3, the MTF-02P range/flow sensor port. The official Configurator resets it on Save; here lines are read-only unless you unlock editing, and only changed lines are sent.'),
  h('div',{style:'margin-bottom:10px'},h('button',{onclick:async()=>{const on=!S.unlock;const r=await BF.try('serial.unlock',{on});if(r!==null||S.demo){S.unlock=on;BF.emit('rerender')}},class:S.unlock?'red':''},S.unlock?'Lock serial editing':'Unlock serial editing'),S.unlock?h('span',{class:'muted'},'  editing enabled: change a field, then Apply'):null));
 const rows=(S.lines.serial||[]).map(l=>{const w=l.split(/\s+/),id=w[1],mask=+w[2],fns=Object.keys(FN).filter(b=>mask&b).map(b=>FN[b]);const key='serial '+id;
  const cur=(S.lineChanges[key]||l).split(/\s+/);const cell=i=>S.unlock?h('input',{type:'text',size:i==2?8:7,value:cur[i],oninput:e=>{const n=cur.slice();n[i]=e.target.value;cur[i]=e.target.value;BF.setLine(key,n.join(' '),l)}}):cur[i];
  return h('tr',{class:key in S.lineChanges?'changed':''},h('td',{},h('b',{},uartName(id)),' ('+id+')'),h('td',{},cell(2)),h('td',{},fns.join(', ')||'-'),h('td',{},cell(3)),h('td',{},cell(4)),h('td',{},cell(5)),h('td',{},cell(6)))});
 el.append(sect('Serial lines (from CLI "serial")',h('table',{class:'t'},h('thead',{},h('tr',{},['Port','Function mask','Functions','MSP baud','GPS baud','Telemetry baud','Blackbox baud'].map(x=>h('th',{},x)))),h('tbody',{},rows))))}};
// ---------- form pages
const fp=(label,icon,secs,extra)=>({label,icon,mode:'settings',render(el){el.append(title(label),extra?extra():null,h('div',{class:'cols'},secs.map(s=>sect(s[0],fields(s[1],s[2])))))}});
P.config=fp('Configuration','M12 8a4 4 0 100 8 4 4 0 000-8zM12 2v3M12 19v3M2 12h3M19 12h3M5 5l2 2M17 17l2 2M5 19l2-2M17 7l2-2',[
 ['Motor / ESC',['motor_pwm_protocol','motor_pwm_rate','dshot_idle_value','dshot_bidir','motor_poles','motor_output_limit','dyn_idle_min_rpm']],
 ['System',['pid_process_denom','gyro_sync_denom','small_angle','airmode_start_throttle_percent','runaway_takeoff_prevention','name','debug_mode']],
 ['Board alignment / trims',['align_board_roll','align_board_pitch','align_board_yaw','acc_trim_roll','acc_trim_pitch']],
 ['Sensors',['acc_hardware','baro_hardware','mag_hardware','rangefinder_hardware','opticalflow_hardware']],
 ['Gyro filters',['gyro_lpf1_type','gyro_lpf1_static_hz','gyro_lpf2_type','gyro_lpf2_static_hz','dyn_notch_count','dyn_notch_q','dyn_notch_min_hz','dyn_notch_max_hz','rpm_filter_harmonics']]],
 ()=>{const feats=(S.lines.feature||[]);const names=[...new Set(feats.map(x=>x.split(' ')[1].replace('-','')))];
  const en=n=>feats.some(x=>x==='feature '+n);return sect('Features (CLI "feature")',h('div',{style:'display:flex;flex-wrap:wrap;gap:4px 18px'},names.map(n=>{const k='feature '+n,on0=en(n);const cb=h('input',{type:'checkbox',checked:k in S.lineChanges?!on0:on0,onchange:e=>{const want=e.target.checked;BF.setLine(k,want===on0?'':(want?'feature ':'feature -')+n,'')}});
   return h('label',{},cb,' '+n)})))});
P.power=fp('Power & Battery','M6 7h11v10H6zM17 10h2v4h-2M9 10v4M12 10v4',[
 ['Voltage',['vbat_scale','vbat_max_cell_voltage','vbat_min_cell_voltage','vbat_warning_cell_voltage','vbat_hysteresis','vbat_detect_cell_voltage']],
 ['Current / capacity',['ibata_scale','ibata_offset','ibat_scale','battery_capacity']]],
 ()=>h('div',{},note('Cell voltages are in 0.01 V units (430 = 4.30 V). After changing vbat_scale, re-check batt_ref_v. The battery only WARNS: it never blocks arming or auto-lands.')));
P.failsafe=fp('Failsafe','M12 3l10 18H2zM12 10v5M12 18v.5',[
 ['Failsafe timing',['failsafe_delay','failsafe_off_delay','failsafe_recovery_delay','failsafe_throttle_low_delay','failsafe_stick_threshold']],
 ['Failsafe action',['failsafe_procedure','failsafe_switch_mode','failsafe_throttle']],['Invalid-pulse detection',['failsafe_min_usec','failsafe_max_usec','rx_min_usec','rx_max_usec']]]);
const ax=['roll','pitch','yaw'];
P.pid={label:'PID Tuning',icon:'M3 18c4 0 3-12 7-12s3 12 7 12 2-6 4-6',mode:'settings',render(el){
 const cell=(n,max)=>{const w=h('span');const f=field(n,{slider:true,smax:max});const inner=f.querySelector('.rng');return inner||h('span',{class:'muted'},'-')};
 const tbl=(hd,rowsN)=>h('table',{class:'t'},h('thead',{},h('tr',{},[h('th',{},'')].concat(hd.map(x=>h('th',{},x))))),h('tbody',{},ax.map(a=>h('tr',{class:rowsN(a).some(n=>n in S.changes)?'changed':''},h('td',{},h('b',{},a[0].toUpperCase()+a.slice(1))),rowsN(a).map(n=>h('td',{},cell(n,n.startsWith('f_')?500:250)))))));
 el.append(title('PID Tuning'),note('PID profile '+(S.profile??0)+'. Values are written diff-only; nothing is sent until you press Apply.'),
  sect('PID',tbl(['P','I','D','D Max (d_min)','Feedforward'],a=>['p_'+a,'i_'+a,'d_'+a,'d_min_'+a,'f_'+a])),
  sect('Rates',field('rates_type'),tbl(['RC rate','Super rate','Expo'],a=>[a+'_rc_rate',a+'_srate',a+'_expo'])),
  h('div',{class:'cols'},sect('PID options',fields(['anti_gravity_gain','iterm_relax','iterm_relax_type','iterm_relax_cutoff','iterm_windup','d_min_gain','d_min_advance','feedforward_transition','throttle_boost'])),
   sect('TPA / limits',fields(['tpa_rate','tpa_breakpoint','throttle_limit_percent','simplified_pids_mode','simplified_master_multiplier']))))}};
// ---------- Receiver
P.rx={label:'Receiver',icon:'M5 12a10 10 0 0114 0M8 15a6 6 0 018 0M12 19v.5',mode:'live',render(el){
 const names=['Roll','Pitch','Throttle','Yaw','AUX1','AUX2','AUX3','AUX4','AUX5','AUX6','AUX7','AUX8','AUX9','AUX10','AUX11','AUX12'];
 const rows=names.map(n=>{const i=h('i'),s=h('span',{},'1500');return {n,i,s,r:h('div',{class:'brow'},h('label',{},n),h('div',{class:'bar'},i,s))}});
 const map=(S.lines.other||[]).find(x=>x.startsWith('map '));
 el.append(title('Receiver'),h('div',{class:'cols'},sect('Live channels (1000 to 2000 us)',rows.map(r=>r.r)),
  h('div',{},sect('Channel map',S.loaded?[h('div',{},h('b',{},'Current: '),map?map.slice(4):'unknown'),h('div',{class:'muted'},'Edit "map" in the CLI or All settings tab (needs settings mode).')]
   :h('span',{class:'muted'},'Channel map is read from the CLI dump; open any settings tab once to load it.')),
   sect('Receiver settings',S.loaded?fields(['serialrx_provider','rssi_channel','rc_deadband','yaw_deadband','mid_rc','min_check','max_check','rx_min_usec','rx_max_usec']):h('span',{class:'muted'},'Available in settings mode.')))));
 BF.liveHook(()=>{const rc=live().rc||[];rows.forEach((r,i)=>{const v=rc[i]??1500;r.i.style.width=Math.max(0,Math.min(100,(v-900)/12))+'%';r.s.textContent=v})})}};
// ---------- Modes
const MODES={0:'ARM',1:'ANGLE',2:'HORIZON',3:'ANTI GRAVITY',4:'MAG',5:'HEADFREE',6:'HEADADJ',7:'CAMSTAB',8:'PASSTHRU',9:'BEEPER',10:'LED LOW',11:'CALIBRATE',12:'OSD DISABLE',13:'TELEMETRY',17:'BLACKBOX',18:'FAILSAFE',19:'AIRMODE',20:'3D',26:'FLIP OVER AFTER CRASH',27:'PREARM'};
P.modes={label:'Modes',icon:'M4 7h16M4 12h16M4 17h16M9 5v4M15 10v4M8 15v4',mode:'settings',render(el){
 el.append(title('Modes'),note('Each row is one CLI "aux" line (mode, AUX channel, range 900 to 2100). Only edited lines are sent.'));
 const lines=S.lines.aux||[],box=h('div');
 lines.forEach(l=>{const w=l.split(/\s+/),idx=w[1],key='aux '+idx;const cur=(S.lineChanges[key]||l).split(/\s+/).slice();const orig=l;
  const used=+cur[5]>+cur[4]&&true;if(!(+w[5]>+w[4])&&!(key in S.lineChanges))return;
  const push=()=>{BF.setLine(key,cur.join(' '),orig);row.classList.toggle('changed',key in S.lineChanges);lab.textContent=cur[4]+' to '+cur[5]};
  const lab=h('span',{style:'width:110px'},cur[4]+' to '+cur[5]);
  const s1=h('input',{type:'range',min:900,max:2100,step:25,value:cur[4],oninput:e=>{cur[4]=Math.min(e.target.value,cur[5]);e.target.value=cur[4];push()}});
  const s2=h('input',{type:'range',min:900,max:2100,step:25,value:cur[5],oninput:e=>{cur[5]=Math.max(e.target.value,cur[4]);e.target.value=cur[5];push()}});
  const mode=h('select',{onchange:e=>{cur[2]=e.target.value;push()}},Object.entries(MODES).map(([k,v])=>h('option',{value:k,selected:k===cur[2]},v)));
  const chan=h('select',{onchange:e=>{cur[3]=e.target.value;push()}},Array.from({length:12},(_,i)=>h('option',{value:i,selected:String(i)===cur[3]},'AUX '+(i+1))));
  const row=h('div',{class:'frow'+(key in S.lineChanges?' changed':'')},h('b',{style:'width:30px'},'#'+idx),mode,chan,h('span',{class:'rng'},s1,s2),lab,
   h('button',{title:'Disable this range',onclick:()=>{cur[4]=900;cur[5]=900;BF.setLine(key,cur.join(' '),orig);BF.emit('rerender')}},'Remove'));box.append(row)});
 const free=lines.find(l=>{const w=l.split(/\s+/);return !(+w[5]>+w[4])&&!(('aux '+w[1]) in S.lineChanges)});
 el.append(sect('Mode ranges',box.children.length?box:h('span',{class:'muted'},'No ranges configured.'),free?h('div',{style:'margin-top:8px'},h('button',{onclick:()=>{const w=free.split(/\s+/);BF.setLine('aux '+w[1],`aux ${w[1]} 1 0 1300 1700 0 0`,free);BF.emit('rerender')}},'Add range')):null))}};
// ---------- Motors
P.motors={label:'Motors',icon:'M12 12m-3 0a3 3 0 106 0 3 3 0 10-6 0M12 9V3M12 15v6M9 12H3M15 12h6',mode:'live',render(el){
 const rows=[1,2,3,4].map(n=>({i:h('i'),s:h('span',{},'-'),n}));
 el.append(title('Motors'),note('Motor test is NOT available in this configurator (project rule: props stay on). Outputs below are read-only.','red'),
  sect('Motor outputs (live, 1000 to 2000)',rows.map(r=>h('div',{class:'brow'},h('label',{},'Motor '+r.n),h('div',{class:'bar'},r.i,r.s)))),
  S.loaded?sect('Motor settings',fields(['motor_pwm_protocol','motor_pwm_rate','dshot_idle_value','dshot_bidir','motor_poles'])):null);
 BF.liveHook(()=>{const m=live().motors||[];rows.forEach((r,i)=>{const v=m[i]??1000;r.i.style.width=Math.max(0,Math.min(100,(v-1000)/10))+'%';r.s.textContent=v})})}};
// ---------- Sensors
const graph=(cv,bufs,min,max,cols)=>{const c=cv.getContext('2d'),W=cv.width,Hh=cv.height;c.fillStyle='#1b1b1b';c.fillRect(0,0,W,Hh);c.strokeStyle='#444';c.beginPath();c.moveTo(0,Hh/2);c.lineTo(W,Hh/2);c.stroke();
 bufs.forEach((b,k)=>{c.strokeStyle=cols[k];c.beginPath();b.forEach((v,i)=>{const x=i*W/150,y=Hh-(v-min)/(max-min)*Hh;i?c.lineTo(x,y):c.moveTo(x,y)});c.stroke()});c.fillStyle='#999';c.font='10px sans-serif';c.fillText(max,3,10);c.fillText(min,3,Hh-3)};
P.sensors={label:'Sensors',icon:'M3 12h4l2-6 4 12 2-6h6',mode:'live',render(el){
 const mk=()=>h('canvas',{width:560,height:170}),g=mk(),a=mk(),r=mk();const B={g:[[],[],[]],a:[[],[],[]],r:[[]]};const col=['#ff5555','#55dd55','#5599ff'];
 el.append(title('Sensors'),h('div',{class:'cols'},sect('Gyroscope (deg/s)  roll red / pitch green / yaw blue',g),sect('Accelerometer (g)  x red / y green / z blue',a),sect('Rangefinder (m)  MTF-02P',r,h('div',{id:'rv',class:'muted'},'-'))));
 BF.liveHook(()=>{const l=live();const push=(b,v)=>{b.push(v);if(b.length>150)b.shift()};(l.gyro||[0,0,0]).forEach((v,i)=>push(B.g[i],v));(l.acc||[0,0,1]).forEach((v,i)=>push(B.a[i],v));if(l.range_cm!=null)push(B.r[0],l.range_cm/100);
  graph(g,B.g,-50,50,col);graph(a,B.a,-2,2,col);graph(r,B.r,0,3,['#ffbb00']);if(l.range_cm!=null)el.querySelector('#rv').textContent=(l.range_cm/100).toFixed(2)+' m'})}};
// ---------- OSD / Blackbox
const prefix=(p,label,icon,hint)=>({label,icon,mode:'settings',render(el){const names=Object.keys(S.values).filter(n=>n.startsWith(p));
 el.append(title(label),hint?note(hint):null,sect(label+' settings ('+names.length+')',names.length?fields(names):h('span',{class:'muted'},'No settings found.')))}});
P.osd=prefix('osd_','OSD','M3 5h18v12H3zM7 21h10M7 9h4M7 13h8','Positions are packed numbers (CLI osd_*_pos). Visual layout editing is not provided: use these values.');
P.blackbox=prefix('blackbox_','Blackbox','M5 4h14v16H5zM8 8h8M8 12h8M8 16h4','Blackbox logging device and rate. Flash/SD erase is not provided here.');
// ---------- CLI
P.cli={label:'CLI',icon:'M4 6l6 6-6 6M12 18h8',mode:'settings',render(el){
 const term=h('div',{id:'term'},'Betaflight CLI. Type "help". Warning: "exit" and "save" REBOOT the FC.\n');const hist=BF.cliHist=BF.cliHist||[];let hi=hist.length;
 if(BF.cliText)term.textContent=BF.cliText;
 const inp=h('input',{id:'cliin',placeholder:'command, Enter to send (Up/Down = history)',autocomplete:'off',spellcheck:false,onkeydown:async e=>{
  if(e.key==='ArrowUp'){if(hi>0)inp.value=hist[--hi]||'';e.preventDefault()}else if(e.key==='ArrowDown'){hi=Math.min(hi+1,hist.length);inp.value=hist[hi]||'';e.preventDefault()}
  else if(e.key==='Enter'&&inp.value.trim()){const l=inp.value;hist.push(l);hi=hist.length;inp.value='';await BF.try('cli.send',{line:l})}}});
 BF.cliOut=t=>{BF.cliText=(BF.cliText||term.textContent)+t+'\n';if(term.isConnected){term.textContent=BF.cliText;term.scrollTop=1e9}};
 el.append(title('CLI'),note('Leaving the CLI with exit or save reboots the FC. Use "Back to live (reboot)" at the bottom to return.'),term,inp);BF.try('cli.open');setTimeout(()=>inp.focus(),0)}};
BF.on('cli',d=>BF.cliOut&&BF.cliOut(d.text));
// ---------- All settings
P.all={label:'All settings',icon:'M4 6h16M4 12h16M4 18h16M8 4v4M15 10v4M10 16v4',mode:'settings',render(el){
 const list=h('div'),cnt=h('span',{class:'muted'});const q=h('input',{class:'search',type:'search',placeholder:'Search settings by name or value...',value:BF.q||'',oninput:e=>{BF.q=e.target.value;draw()}});
 const draw=()=>{const t=(BF.q||'').toLowerCase().trim();let n=Object.keys(S.values).filter(k=>!t||k.includes(t)||(S.values[k]).toLowerCase().includes(t)||(S.groups[k]||'').includes(t));n.sort((a,b)=>(a in S.changes?0:1)-(b in S.changes?0:1)||a.localeCompare(b));
  const tot=n.length;n=n.slice(0,150);cnt.textContent=tot+' match'+(tot==1?'':'es')+(tot>150?' (showing 150, refine search)':'');list.replaceChildren(...n.map(k=>{const f=field(k,{slider:true});f.addEventListener('focusin',()=>BF.meta(k),{once:true});return f}));
  if(tot&&tot<=25)n.forEach(k=>BF.meta(k))};
 el.append(title('All settings'),h('div',{class:'split'},h('div',{},q,cnt,sect('Settings',list)),BF.changesPanel()));draw()}};
BF.order=['setup','ports','config','power','failsafe','pid','rx','modes','motors','sensors','osd','blackbox','cli','all'];
BF.tabAlias={configuration:'config',receiver:'rx',pid:'pid',allsettings:'all',battery:'power'};
})();
