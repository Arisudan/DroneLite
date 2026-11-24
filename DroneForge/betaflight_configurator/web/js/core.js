// Core: DOM helper, RPC, state, banners, settings-bound fields, apply/save.
(function(){
const BF=window.BF={pages:{},liveCbs:[],H:{}};
const S=BF.S={values:{},groups:{},lines:{},changes:{},lineChanges:{},meta:{},info:null,connected:false,live:null,loaded:false,unlock:false,demo:false,mode:'offline'};
const h=BF.h=(tag,attrs,...kids)=>{const e=document.createElement(tag);
 for(const[k,v]of Object.entries(attrs||{})){if(v==null||v===false)continue;if(k.startsWith('on'))e.addEventListener(k.slice(2),v);else if(k==='class')e.className=v;else if(k in e&&k!=='list')try{e[k]=v}catch(_){e.setAttribute(k,v)}else e.setAttribute(k,v)}
 const add=c=>{if(Array.isArray(c))c.forEach(add);else if(c!=null&&c!==false)e.append(c.nodeType?c:document.createTextNode(c))};kids.forEach(add);return e};
BF.sect=(title,...k)=>h('div',{class:'box'},h('div',{class:'box-title'},title),h('div',{class:'box-body'},k));
BF.note=(t,cls)=>h('div',{class:'note '+(cls||'')},t);
// ---- RPC
let ws,nid=1;const pend={},hand={};
BF.on=(e,f)=>(hand[e]=hand[e]||[]).push(f);BF.emit=(e,d)=>(hand[e]||[]).forEach(f=>f(d));
BF.call=(m,p)=>new Promise((ok,no)=>{const id=nid++;pend[id]={ok,no};ws.send(JSON.stringify({id,method:m,params:p||{}}))});
BF.try=async(m,p)=>{try{return await BF.call(m,p)}catch(e){BF.banner('error',String(e&&e.message||e));return null}};
function wire(w){w.onmessage=ev=>{const m=JSON.parse(ev.data);if(m.event)BF.emit(m.event,m.data);else if(pend[m.id]){const p=pend[m.id];delete pend[m.id];m.error!=null?p.no(m.error):p.ok(m.result)}}}
BF.start=()=>new Promise(ok=>{const q=new URLSearchParams(location.search);
 const demo=()=>{S.demo=true;ws=new window.DemoWS();wire(ws);document.getElementById('demoTag').hidden=false;if(!q.has('demo'))BF.banner('info','No backend found: running the built-in DEMO backend. Nothing is sent to a flight controller.');ok()};
 if(q.get('demo')==='1')return demo();
 const url=location.protocol.startsWith('http')?'ws://'+location.host+'/ws':'ws://127.0.0.1:8765/ws';let done=false;
 const t=setTimeout(()=>{if(!done){done=true;try{w.close()}catch(_){}demo()}},1000);let w;
 try{w=new WebSocket(url)}catch(_){clearTimeout(t);done=true;return demo()}
 w.onopen=()=>{if(done)return;done=true;clearTimeout(t);ws=w;wire(w);w.onclose=()=>BF.banner('error','Connection to the local server lost. Reload the page.');ok()};
 w.onerror=()=>{if(!done){done=true;clearTimeout(t);demo()}}});
// ---- banners (no modals)
BF.banner=(lvl,text,act)=>{const box=document.getElementById('banners');
 for(const b of box.children)if(b.dataset.t===text)return;
 const b=h('div',{class:'banner '+lvl,'data-t':text},h('span',{},text),act&&h('button',{onclick:()=>{act.fn();b.remove()}},act.label),h('button',{class:'x',onclick:()=>b.remove(),title:'Dismiss'},'×'));
 box.append(b);if(lvl==='info')setTimeout(()=>b.remove(),9000)};
BF.clearBanners=()=>document.getElementById('banners').replaceChildren();
// ---- settings state
BF.val=n=>n in S.changes?S.changes[n]:S.values[n];
BF.meta=n=>S.meta[n]||(S.meta[n]=BF.call('settings.meta',{name:n}).catch(e=>{delete S.meta[n];return null}));
BF.pendingCount=()=>Object.keys(S.changes).length+Object.keys(S.lineChanges).length;
BF.setChange=(n,v)=>{v=String(v);if(v===S.values[n])delete S.changes[n];else S.changes[n]=v;BF.emit('changes')};
BF.setLine=(key,line,orig)=>{if(line===orig)delete S.lineChanges[key];else S.lineChanges[key]=line;BF.emit('changes')};
BF.revert=(k)=>{if(k in S.changes)delete S.changes[k];else delete S.lineChanges[k];BF.emit('changes');BF.emit('rerender')};
BF.changeList=()=>[...Object.keys(S.changes).map(n=>({k:n,name:n,old:S.values[n],nw:S.changes[n]})),
 ...Object.keys(S.lineChanges).map(k=>({k,name:k,old:(origLine(k)||'(none)'),nw:S.lineChanges[k]}))];
function origLine(k){const g=k.split(' ')[0];const a=S.lines[g]||[];return k.startsWith('feature ')?a.find(x=>x.replace('-','')===('feature '+k.split(' ')[1]))||'':a.find(x=>x.split(' ').slice(0,2).join(' ')===k)}
BF.loadDump=async(force)=>{if(S.loaded&&!force)return true;const d=await BF.try('settings.dump');if(!d)return false;
 S.values={};S.groups={};d.settings.forEach(s=>{S.values[s.name]=String(s.value);S.groups[s.name]=s.group});S.lines=d.lines||{};S.version_line=d.version_line;S.loaded=true;return true};
BF.apply=async()=>{if(!BF.pendingCount())return true;const r=await BF.try('settings.apply',{changes:{...S.changes},lines:Object.values(S.lineChanges)});if(!r)return false;
 (r.applied||[]).forEach(a=>{if(a.verified===false)BF.banner('warn','Could not verify '+a.name+' after set (expected '+a.new+')');delete S.changes[a.name]});
 (r.failed||[]).forEach(f=>BF.banner('error','Failed to set '+f.name+': '+f.error));
 if(!(r.failed||[]).length){S.changes={};S.lineChanges={}}
 await BF.loadDump(true);if(!(r.failed||[]).length)BF.banner('info','Applied '+(r.applied||[]).length+' change(s) (not saved yet). Backup: '+(r.backup||'-'));
 BF.emit('changes');BF.emit('rerender');return !(r.failed||[]).length};
BF.save=async()=>{if(BF.pendingCount()&&!await BF.apply())return;BF.banner('info','Saving: the FC reboots, then the app returns to LIVE...');const r=await BF.try('settings.save');if(r){S.loaded=false;S.changes={};S.lineChanges={};if(r.info)BF.setInfo(r.info);BF.emit('saved');BF.banner('info','Saved. FC rebooted, mode LIVE.')}};
// ---- fields
BF.field=(name,o={})=>{
 const lab=h('label',{title:name},o.label||name);
 if(!(name in S.values))return h('div',{class:'frow'},lab,h('span',{class:'muted'},'(not present on this firmware)'));
 const orig=S.values[name],wrap=h('span',{class:'rng'}),hint=h('span',{class:'hint'}),row=h('div',{class:'frow'},lab,wrap,hint,o.unit?h('span',{class:'hint'},o.unit):null);
 const upd=()=>{row.classList.toggle('changed',name in S.changes);wrap.classList.toggle('chgd',name in S.changes)};
 const build=m=>{wrap.replaceChildren();const cur=BF.val(name);let c;
  const set=v=>{BF.setChange(name,v);upd();hint.textContent=(name in S.changes?'was '+orig+'  ':'')+rng(m)};
  if(m&&m.table){c=h('select',{onchange:e=>set(e.target.value)},m.table.map(t=>h('option',{value:t,selected:t===cur},t)));wrap.append(c)}
  else if(/^-?\d+(\.\d+)?$/.test(orig)){const step=/\./.test(orig)?'0.01':'1';const hasR=m&&m.max!==m.min&&m.max!=null;
   const n=h('input',{type:'number',step,value:cur,min:hasR?m.min:null,max:hasR?m.max:null,oninput:e=>{if(e.target.value==='')return;set(e.target.value);if(sl)sl.value=e.target.value}});
   const sl=o.slider&&hasR?h('input',{type:'range',min:m.min,max:Math.min(m.max,o.smax||m.max),step,value:cur,oninput:e=>{n.value=e.target.value;set(e.target.value)}}):null;if(sl)wrap.append(sl);wrap.append(n)}
  else wrap.append(h('input',{type:'text',value:cur,oninput:e=>set(e.target.value)}));
  hint.textContent=(name in S.changes?'was '+orig+'  ':'')+rng(m);upd()};
 const rng=m=>m?(m.table?'':(m.max!=m.min?'range '+m.min+' to '+m.max:'')+(m.default!=null?'  default '+m.default:'')):'';
 build(null);BF.meta(name).then(m=>{if(m)build(m)});return row};
BF.fields=(names,o)=>names.map(n=>BF.field(n,o));
BF.changesPanel=()=>{const box=h('div',{class:'box-body'}),t=h('div',{class:'box-title'});
 const draw=()=>{const l=BF.changeList();t.textContent='Changes ('+l.length+')';box.replaceChildren(l.length?l.map(c=>h('div',{class:'chg'},h('span',{},c.name),h('s',{},c.old),'→',h('b',{},c.nw),h('button',{title:'Revert',onclick:()=>BF.revert(c.k)},'×'))):h('span',{class:'muted'},'No pending changes.'),
  h('div',{style:'margin-top:8px;display:flex;gap:6px'},h('button',{disabled:!l.length,onclick:()=>BF.apply()},'Apply'),h('button',{class:'orange',onclick:()=>BF.save()},'Save and Reboot')))};
 draw();const f=()=>{if(box.isConnected)draw()};BF.on('changes',f);return h('div',{class:'box sticky'},t,box)};
BF.setInfo=i=>{S.info={...S.info,...i};S.mode=i.mode||S.mode;const $=id=>document.getElementById(id);
 $('s-pkt').textContent=i.packet_errors??0;$('s-i2c').textContent=i.i2c_errors??0;$('s-cyc').textContent=i.cycle_us??0;$('s-cpu').textContent=i.cpu_load??0;
 const m=$('s-mode');const mode=S.connected?S.mode:'offline';m.textContent=mode==='live'?'LIVE':mode==='settings'?'SETTINGS':'OFFLINE';m.className='mode '+(mode==='live'||mode==='settings'?mode:'off');
 $('s-msg').textContent=mode==='settings'?'Settings mode: the FC is in CLI; Save and Reboot returns to live':'';$('liveBtn').hidden=mode!=='settings'};
})();
