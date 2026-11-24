(function(){
const {h,S}=BF,$=id=>document.getElementById(id);let cur=null,navSeq=0;
const tabs=$('tabs');
BF.order.forEach(id=>{const p=BF.pages[id];tabs.append(h('a',{id:'tab-'+id,onclick:()=>go(id)},h('span',{innerHTML:`<svg viewBox="0 0 24 24"><path d="${p.icon}"/></svg>`}),p.label))});
async function go(id){const seq=++navSeq,p=BF.pages[id];if(!p)return;cur=id;BF.liveCbs=[];
 BF.order.forEach(t=>$('tab-'+t).classList.toggle('on',t===id));
 const el=$('content');
 if(!S.connected){el.replaceChildren(h('h1',{class:'pt'},p.label),BF.note('Not connected. Choose a port and press CONNECT.'));return}
 if(p.mode==='settings'){if(S.mode!=='settings'){const r=await BF.try('mode.settings');if(r)BF.setInfo(r);else return;if(seq!==navSeq)return}
  if(!await BF.loadDump()||seq!==navSeq)return}
 else if(S.mode==='settings'){BF.banner('warn','Live tabs need LIVE mode. The FC is in settings mode (CLI); click "Back to live (reboot)" at the bottom.',{label:'Back to live (reboot)',fn:backLive});if(S.changes&&BF.pendingCount())BF.banner('warn','You have unapplied changes: Apply them before going live.')}
 render()}
function render(){const p=BF.pages[cur];if(!p)return;const el=$('content'),sc=el.scrollTop;el.replaceChildren();BF.liveCbs=[];try{p.render(el)}catch(e){console.error(e);BF.banner('error','UI error in '+p.label+': '+e.message)}el.scrollTop=sc;BF.emit('changes')}
BF.on('rerender',()=>{if(S.connected&&cur&&$('content'))render()});
async function backLive(){const r=await BF.try('mode.live');if(r){S.loaded=false;BF.setInfo(r);BF.banner('info','Back in LIVE mode.');if(cur)go(cur)}}
BF.on('status',d=>{S.connected=d.connected!==false;BF.setInfo(d)});
BF.on('live',d=>{S.live=d;BF.liveCbs.forEach(f=>f())});
BF.on('saved',()=>go('setup'));   // after Save the FC is back in LIVE: open the live Setup tab (a settings tab would re-enter CLI and reboot again)
BF.on('changes',()=>{const n=BF.pendingCount();$('pending').textContent=n?n+' unsaved change'+(n>1?'s':'')+' (not yet applied to the FC)':'';$('applyBtn').disabled=!n;$('applyBtn').textContent='Apply changes'+(n?' ('+n+')':'')});
async function refreshPorts(){const l=await BF.try('ports.list')||[];const s=$('port');s.replaceChildren(...l.map(p=>h('option',{value:p.path,selected:p.kind==='betaflight'},p.description+(p.kind==='radio'?' (radio, do not use)':'')+' '+p.path.split('/').pop())));if(!l.length)s.append(h('option',{value:''},'No ports found'))}
async function connect(){if(S.connected){await BF.try('fc.disconnect');S.connected=false;S.loaded=false;S.mode='offline';BF.setInfo({});$('connect').textContent='CONNECT';$('connect').className='orange';if(cur)go(cur);return}
 const port=$('port').value;if(!port){BF.banner('error','No port selected.');return}
 const i=await BF.try('fc.connect',{port,baud:+$('baud').value});if(!i)return;S.connected=true;S.mode=i.mode||'live';BF.setInfo(i);$('connect').textContent='DISCONNECT';$('connect').className='orange';BF.clearBanners();if(S.demo)BF.banner('info','DEMO mode: simulated flight controller.');go(cur||'setup')}
$('refresh').onclick=refreshPorts;$('connect').onclick=connect;$('liveBtn').onclick=backLive;
$('applyBtn').onclick=()=>BF.apply();$('saveBtn').onclick=()=>BF.save();
$('rebootBtn').onclick=async()=>{if(BF.pendingCount())BF.banner('warn','Unapplied changes will be lost on reboot.');if(await BF.try('fc.reboot',{target:'normal'})){S.loaded=false;S.mode='live';BF.setInfo({mode:'live'});BF.banner('info','Reboot sent. FC returns in LIVE mode.');if(cur)go(cur)}};
(async()=>{await BF.start();BF.emit('changes');await refreshPorts();const q=new URLSearchParams(location.search);let t=(q.get('tab')||'setup').toLowerCase();t=BF.tabAlias[t]||t;if(!BF.pages[t])t='setup';cur=t;
 if(S.demo){await connect()}else go(t)})();
})();
