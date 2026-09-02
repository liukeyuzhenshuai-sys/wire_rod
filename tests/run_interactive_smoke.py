from __future__ import annotations
from pathlib import Path
import json,sys,time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tracker.engine import load_config
from tracker.session import SessionManager

cfg=load_config(ROOT/'tracker_config.json'); sm=SessionManager(ROOT,cfg)
sess,_=sm.start('S03')
while sess.current_index < len(sess.frames)-1:
    sess,_,_=sm.next(sess.session_id)
seed=sm.editor_seed(sess.session_id)
existing=[]
for k,r in enumerate(seed['objects']):
    existing.append({'global_id':r['global_id'],'dx':.025*(k+1),'dy':.008*((k%2)*2-1),'dz':0.0,'dyaw_deg':(-1)**k*(1.0+.25*k),
                     'observation':'roll_mild' if k%3==0 else 'normal','occlusion_side':'both' if k==2 else 'none','severity':.82 if k==2 else 0.0})
left=[r for r in seed['objects'] if r.get('cluster')=='left']; x=max(r['center'][0] for r in left)+1.72
req={'seed':424242,'existing':existing,'new_coils':[{'prototype':'prototype_04','diameter':1.35,'length':1.17,'x':x,'y':.02,'z':.68,'yaw_deg':6.0,'layer':1,
      'observation':'roll_mild','occlusion_side':'left','severity':.40,'hint_dx':-.15,'hint_dy':.04,'hint_dz':0.0}]}
t0=time.perf_counter(); sess,created,ev=sm.create_next(sess.session_id,req); elapsed=(time.perf_counter()-t0)*1000
assert ev['wrong_id_count']==0,ev
assert all(int(r['point_count'])>0 for r in created['gt']), 'occlusion/full hide invariant violated'
assert Path(sess.current_file).parent==Path('.'), sess.current_file
partial=[r for r in created['gt'] if r['visibility'] in ('PARTIAL_VISIBLE','OCCLUDED')]
assert partial and all(int(r['point_count'])>0 for r in partial)
out={'pass':True,'sequence':'S03','created_file':sess.current_file,'generated_objects':len(created['gt']),'partial_objects':len(partial),'wrong_id':ev['wrong_id_count'],'elapsed_ms':round(elapsed,3),'warnings':created['warnings']}
(ROOT/'results'/'interactive_v0_4_smoke.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(out,ensure_ascii=False,indent=2))
# Keep repository/package clean; the Web creates USER_*.ply at runtime.
(ROOT/sess.current_file).unlink(missing_ok=True)
