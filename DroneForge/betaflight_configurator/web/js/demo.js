// Built-in fake backend implementing the CONTRACT.md WebSocket protocol (no server needed).
(function(){
const S=[];
const add=(n,v,g,min,max,t)=>S.push({name:n,value:String(v),group:g,min,max,table:t||null,default:String(v)});
const T={proto:['OFF','ONESHOT125','ONESHOT42','MULTISHOT','BRUSHED','DSHOT150','DSHOT300','DSHOT600'],
 onoff:['OFF','ON'],rates:['BETAFLIGHT','RACEFLIGHT','KISS','ACTUAL','QUICK'],fsproc:['AUTO-LAND','DROP','GPS-RESCUE'],
 fsmode:['STAGE1','STAGE2'],lpf:['PT1','BIQUAD','PT2','PT3'],rxp:['SPEK1024','SPEK2048','SBUS','SUMD','IBUS','CRSF'],
 dbg:['NONE','CYCLETIME','BATTERY','GYRO_FILTERED','GYRO_SCALED','RC_INTERPOLATION','RANGEFINDER'],
 bbdev:['NONE','SPIFLASH','SDCARD','SERIAL'],bbmode:['NORMAL','MOTOR_TEST','ALWAYS'],units:['IMPERIAL','METRIC','BRITISH'],
 irelax:['OFF','ON'],irt:['GYRO','SETPOINT'],accHw:['AUTO','NONE','MPU6000','ICM42688P'],baro:['AUTO','NONE','BMP280','DPS310']};
// config
[['motor_pwm_protocol','DSHOT600',T.proto],['motor_pwm_rate',480,0,200,32000],['dshot_idle_value',550,0,0,2000],['dshot_bidir','OFF',T.onoff],['motor_poles',14,0,2,100],['motor_output_limit',100,0,1,100],
['pid_process_denom',2,0,1,16],['small_angle',25,0,0,180],['acc_trim_pitch',0,0,-300,300],['acc_trim_roll',0,0,-300,300],['align_board_roll',0,0,-180,360],['align_board_pitch',0,0,-180,360],['align_board_yaw',0,0,-180,360],
['gyro_lpf1_type','PT1',T.lpf],['gyro_lpf1_static_hz',250,0,0,1000],['gyro_lpf2_type','PT1',T.lpf],['gyro_lpf2_static_hz',500,0,0,1000],['dyn_notch_count',3,0,0,5],['dyn_notch_q',300,0,1,1000],['dyn_notch_min_hz',100,0,20,250],['dyn_notch_max_hz',600,0,200,1000],
['rpm_filter_harmonics',0,0,0,3],['debug_mode','NONE',T.dbg],['acc_hardware','AUTO',T.accHw],['baro_hardware','AUTO',T.baro],['mag_hardware','NONE',T.accHw],['gyro_sync_denom',1,0,1,32],['name','DRONEFORGE',0,0,16],['airmode_start_throttle_percent',25,0,0,100],['runaway_takeoff_prevention','ON',T.onoff],['dyn_idle_min_rpm',0,0,0,200],['thr_corr_value',0,0,0,150],['launch_control_mode','NORMAL',['NORMAL','PITCHONLY','FULL']],['ez_landing_threshold',25,0,0,200],['rc_smoothing','ON',T.onoff],['rc_smoothing_auto_factor',30,0,0,250]].forEach(a=>add(a[0],a[1],'config',a[3],a[4],Array.isArray(a[2])?a[2]:null));
// power
[['vbat_scale',110,'power',0,255],['vbat_max_cell_voltage',430,'power',100,500],['vbat_min_cell_voltage',330,'power',50,500],['vbat_warning_cell_voltage',350,'power',50,500],['vbat_hysteresis',1,'power',0,250],['ibata_scale',400,'power',-16000,16000],['ibata_offset',0,'power',-32000,32000],['battery_capacity',1500,'power',0,20000],['vbat_detect_cell_voltage',300,'power',0,2000],['ibat_scale',400,'power',-16000,16000]].forEach(a=>add(...a));
// failsafe
[['failsafe_delay',15,'failsafe',0,200],['failsafe_off_delay',10,'failsafe',0,200],['failsafe_throttle',1000,'failsafe',1000,2000],['failsafe_switch_mode','STAGE1','failsafe',0,0,T.fsmode],['failsafe_procedure','AUTO-LAND','failsafe',0,0,T.fsproc],['failsafe_recovery_delay',5,'failsafe',1,200],['failsafe_stick_threshold',30,'failsafe',0,50],['failsafe_throttle_low_delay',100,'failsafe',0,300],['failsafe_min_usec',885,'failsafe',100,2000],['failsafe_max_usec',2115,'failsafe',100,2000]].forEach(a=>add(a[0],a[1],a[2],a[3],a[4],a[5]));
// rx
[['serialrx_provider','CRSF','rx',0,0,T.rxp],['rssi_channel',0,'rx',0,18],['rc_deadband',0,'rx',0,32],['yaw_deadband',0,'rx',0,100],['rx_min_usec',885,'rx',750,2250],['rx_max_usec',2115,'rx',750,2250],['mid_rc',1500,'rx',1200,1700],['min_check',1050,'rx',750,2250],['max_check',1900,'rx',750,2250],['min_throttle',1070,'rx',0,2000],['max_throttle',2000,'rx',0,2000],['stick_arming_threshold',0,'rx',0,500]].forEach(a=>add(a[0],a[1],a[2],a[3],a[4],a[5]));
// pid + rates
const ax=['roll','pitch','yaw'],P={roll:[48,52,46,125],pitch:[52,56,48,130],yaw:[45,60,0,120]};
ax.forEach(a=>{const p=P[a];add('p_'+a,p[0],'pid',0,250);add('i_'+a,p[1],'pid',0,250);add('d_'+a,p[2],'pid',0,250);add('d_min_'+a,Math.round(p[2]*.8),'pid',0,250);add('f_'+a,p[3],'pid',0,2000);
 add(a+'_rc_rate',a=='yaw'?100:120,'rates',1,255);add(a+'_expo',a=='yaw'?0:20,'rates',0,100);add(a+'_srate',a=='yaw'?60:70,'rates',0,100)});
add('rates_type','ACTUAL','rates',0,0,T.rates);
[['anti_gravity_gain',80,'pid',1,250],['iterm_relax','ON','pid',0,0,T.irelax],['iterm_relax_type','GYRO','pid',0,0,T.irt],['iterm_relax_cutoff',15,'pid',1,50],['iterm_windup',85,'pid',30,100],['d_min_gain',27,'pid',0,100],['d_min_advance',20,'pid',0,200],['feedforward_transition',0,'pid',0,100],['throttle_boost',5,'pid',0,100],['tpa_rate',65,'pid',0,100],['tpa_breakpoint',1350,'pid',1000,2000],['throttle_limit_percent',100,'pid',25,100]].forEach(a=>add(a[0],a[1],a[2],a[3],a[4],a[5]));
// blackbox
[['blackbox_device','NONE','blackbox',0,0,T.bbdev],['blackbox_mode','NORMAL','blackbox',0,0,T.bbmode],['blackbox_sample_rate','1/2','blackbox',0,0,['1/1','1/2','1/4','1/8','1/16']],['blackbox_high_resolution','OFF','blackbox',0,0,T.onoff],['blackbox_disable_pids','OFF','blackbox',0,0,T.onoff],['blackbox_disable_rc','OFF','blackbox',0,0,T.onoff],['blackbox_disable_gyro','OFF','blackbox',0,0,T.onoff],['blackbox_disable_motors','OFF','blackbox',0,0,T.onoff]].forEach(a=>add(a[0],a[1],a[2],a[3],a[4],a[5]));
// osd
add('osd_units','METRIC','osd',0,0,T.units);add('osd_warn_dji_compat','OFF','osd',0,0,T.onoff);add('osd_rssi_alarm',20,'osd',0,100);add('osd_cap_alarm',2200,'osd',0,20000);add('osd_alt_alarm',100,'osd',0,10000);add('osd_ah_sbar_pos',1,'osd',0,1);
['rssi','main_batt_voltage','crosshairs','artificial_horizon','horizon_sidebars','current_draw','mah_drawn','craft_name','throttle_pos','vtx_channel','link_quality','flymode','warnings','disarmed','gps_speed','altitude','pid_roll','pid_pitch','pid_yaw','rtc_time','debug','avg_cell_voltage','stat_max_alt','compass_bar'].forEach((o,i)=>add('osd_'+o+'_pos',(i*131+2049)%4000,'osd',0,65535));
// misc
[['gyro_cal_on_first_arm','OFF','other',0,0,T.onoff],['acc_calibration_status','0','other',0,1],['pwr_on_arm_grace',5,'other',0,30],['auto_disarm_delay',5,'other',0,60],['gps_provider','UBLOX','other',0,0,['NMEA','UBLOX','MSP']],['gps_auto_baud','ON','other',0,0,T.onoff],['rangefinder_hardware','MSP','other',0,0,['NONE','MSP','HCSR04']],['opticalflow_hardware','MSP','other',0,0,['NONE','MSP']],['telemetry_inverted','OFF','other',0,0,T.onoff],['crsf_use_negotiated_baud','OFF','other',0,0,T.onoff],['beeper_dshot_beacon_tone',1,'other',1,5],['tlm_halfduplex','ON','other',0,0,T.onoff],['usb_hid_cdc','OFF','other',0,0,T.onoff],['vtx_band',0,'other',0,5],['vtx_channel',0,'other',0,8],['vtx_power',1,'other',0,5],['sdcard_detect_inverted','OFF','other',0,0,T.onoff],['osd_displayport_device','AUTO','other',0,0,['NONE','AUTO','MSP','MAX7456']],['position_hold_sensitivity',50,'other',0,100],['altitude_hold_max_throttle',1700,'other',1000,2000],['pid_audio','OFF','other',0,0,T.onoff],['dashboard_i2c_addr',60,'other',8,119],['simplified_pids_mode','RPY','other',0,0,['OFF','RP','RPY']],['simplified_master_multiplier',100,'other',0,200],['thrust_linear',0,'other',0,150],['mixer_type','LEGACY','other',0,0,['LEGACY','LINEAR','DYNAMIC']]].forEach(a=>add(a[0],a[1],a[2],a[3],a[4],a[5]));
const meta={};S.forEach(s=>meta[s.name]=s);
const lines={serial:['serial 20 1 115200 57600 0 115200','serial 0 0 115200 57600 0 115200','serial 1 64 115200 57600 0 115200','serial 2 262144 115200 57600 0 115200','serial 3 0 115200 57600 0 115200','serial 4 0 115200 57600 0 115200'],
 aux:['aux 0 0 0 1700 2100 0 0','aux 1 1 1 900 1300 0 0','aux 2 2 1 1300 1700 0 0','aux 3 19 2 1700 2100 0 0','aux 4 0 0 900 900 0 0','aux 5 0 0 900 900 0 0','aux 6 0 0 900 900 0 0','aux 7 0 0 900 900 0 0'],
 feature:['feature -RX_PPM','feature -INFLIGHT_ACC_CAL','feature -RX_SERIAL','feature -MOTOR_STOP','feature -SOFTSERIAL','feature -GPS','feature -TELEMETRY','feature -LED_STRIP','feature -OSD','feature -AIRMODE','feature -ANTI_GRAVITY','feature RX_SERIAL','feature AIRMODE','feature OSD','feature ESC_SENSOR'].filter((x,i,a)=>!a.includes(x.replace('-','')) || !x.includes('-')),
 resource:['resource MOTOR 1 B00','resource MOTOR 2 B01','resource BEEPER 1 A15'],other:['map AETR','mcu_id 0023004a3236510d','board_name MATEKH743']};
function DemoWS(){
 const me=this;me.readyState=1;let mode='live',connected=false,t=0,cliOpen=false,vals={};S.forEach(s=>vals[s.name]=s.value);
 const ev=(e,d)=>setTimeout(()=>me.onmessage&&me.onmessage({data:JSON.stringify({event:e,data:d})}),0);
 const info=()=>({api:'1.46',variant:'BTFL',version:'4.5.4',board:'MATEKH743',build:'Nov 10 2026 15:42:40',uid:'0023004a3236510d',armed:false,mode,cycle_us:250,cpu_load:12+Math.round(Math.random()*3),i2c_errors:0,packet_errors:0});
 setInterval(()=>{if(!connected)return;ev('status',{...info(),connected:true})},500);
 setInterval(()=>{if(!connected||mode!=='live')return;t+=.1;const s=Math.sin;
  ev('live',{attitude:{roll:20*s(t*.8),pitch:12*s(t*.55+1),yaw:(t*20)%360},rc:Array.from({length:16},(_,i)=>i<4?Math.round(1500+400*s(t*(.5+i*.2))):(i==4?2000:1000+(i%3)*500)),
  motors:[0,1,2,3].map(i=>Math.round(1100+80*s(t*2+i))),gyro:[s(t*3)*8,s(t*2.2)*6,s(t)*4],acc:[s(t)*.1,s(t*.7)*.1,1+s(t*5)*.01],
  analog:{vbat:16.1+.1*s(t),amps:2.4+s(t),rssi:90+Math.round(5*s(t))},range_cm:Math.round(60+25*s(t*.6))})},100);
 const H={'ports.list':()=>[{path:'/dev/serial/by-id/usb-Betaflight_STM32H743-if00',description:'Betaflight STM32H743 (demo)',kind:'betaflight'},{path:'/dev/ttyUSB0',description:'CP2102 radio',kind:'radio'}],
 'fc.connect':()=>{connected=true;return info()},'fc.disconnect':()=>{connected=false;return {}},'fc.info':info,
 'mode.settings':()=>{mode='settings';return info()},'mode.live':()=>new Promise(r=>setTimeout(()=>{mode='live';cliOpen=false;r(info())},400)),
 'fc.reboot':p=>new Promise(r=>setTimeout(()=>{mode='live';cliOpen=false;r({rebooted:p.target||'normal'})},400)),'fc.calibrate_acc':()=>({started:true}),
 'settings.dump':()=>({version_line:'# Betaflight / STM32H743 (S743) 4.5.4 Nov 10 2026 (demo)',settings:S.map(s=>({name:s.name,value:vals[s.name],group:s.group})),lines:JSON.parse(JSON.stringify(lines)),profile:0,rateprofile:0}),
 'settings.meta':p=>{const m=meta[p.name];if(!m)throw 'Unknown setting '+p.name;return {min:m.min,max:m.max,default:m.default,table:m.table,type:m.table?'lookup':(isNaN(m.value)?'string':'uint16')}},
 'settings.apply':p=>{const r={applied:[],failed:[],backup:'backups/DEMO_before-change.txt'};
  for(const[n,v]of Object.entries(p.changes||{})){const m=meta[n];if(!m){r.failed.push({name:n,error:'Invalid name'});continue}
   if(m.table&&!m.table.includes(v)){r.failed.push({name:n,error:'Invalid value '+v});continue}
   if(!m.table&&!isNaN(v)&&(+v<m.min||+v>m.max)){r.failed.push({name:n,error:'Value '+v+' out of range '+m.min+'..'+m.max});continue}
   if(vals[n]!==String(v))r.applied.push({name:n,old:vals[n],new:String(v),verified:true});vals[n]=String(v)}
  (p.lines||[]).forEach(l=>{const w=l.split(' ');const k=w[0],i=k==='feature'?lines.feature.findIndex(x=>x.replace('-','')===l.replace('-','')):lines[k]?lines[k].findIndex(x=>x.split(' ')[1]===w[1]):-1;if(i>=0)lines[k][i]=l});return r},
 'serial.unlock':({on})=>({serial_unlocked:!!on}),
 'settings.save':()=>new Promise(r=>setTimeout(()=>{mode='live';cliOpen=false;r({saved:true,info:info()})},500)),
 'settings.diff':()=>({text:'# diff all (demo)'}),'settings.export':()=>({text:S.map(s=>'set '+s.name+' = '+vals[s.name]).join('\n')}),
 'cli.open':()=>{cliOpen=true;return {}},'cli.close':()=>{mode='live';return {}},
 'cli.send':p=>{const l=(p.line||'').trim(),w=l.split(/\s+/);let o='# '+l+'\n';
  if(w[0]==='get'){const q=(w[1]||'').toLowerCase();const m=S.filter(s=>s.name.includes(q)).slice(0,20);o+=m.length?m.map(s=>s.name+' = '+vals[s.name]+(s.table?'\nAllowed values: '+s.table.join(', '):'\nAllowed range: '+s.min+' - '+s.max)).join('\n\n'):'Invalid name'}
  else if(w[0]==='set'){const m=l.match(/^set\s+(\S+)\s*=\s*(.+)$/);if(m&&meta[m[1]]){vals[m[1]]=m[2].trim();o+=m[1]+' set to '+m[2].trim()}else o+='Invalid name'}
  else if(w[0]==='version')o+='# Betaflight / STM32H743 (S743) 4.5.4 Nov 10 2026';else if(w[0]==='status')o+='System Uptime: 120 seconds\nCPU:12%, cycle time: 250, GYRO rate: 4000';
  else if(w[0]==='help')o+='Commands: get set aux serial feature status version diff dump save exit';else if(w[0]==='save'||w[0]==='exit'){o+='Rebooting...';mode='live'}else o+='###ERROR IN CLI: Unknown command, try \'help\'';
  ev('cli',{text:o+'\n\n# '});return {}}};
 me.send=s=>{const m=JSON.parse(s);setTimeout(()=>{let res;try{res=H[m.method]?H[m.method](m.params||{}):(()=>{throw 'Unknown method '+m.method})();Promise.resolve(res).then(r=>me.onmessage({data:JSON.stringify({id:m.id,result:r})}),e=>me.onmessage({data:JSON.stringify({id:m.id,error:String(e)})}))}catch(e){me.onmessage({data:JSON.stringify({id:m.id,error:String(e)})})}},20)};
 setTimeout(()=>me.onopen&&me.onopen(),0);
}
window.DemoWS=DemoWS;
})();
