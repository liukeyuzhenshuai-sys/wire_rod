from __future__ import annotations
from pathlib import Path
from functools import lru_cache
import json, sys, time, zipfile, copy
import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from tracker.engine import load_config
from tracker.ply import load_algorithm_input
from tracker.session import SessionManager, neighbor_gaps

app=FastAPI(title='Coil Semantic Tracker V1 - Sequential Inspector')
app.mount('/static',StaticFiles(directory=ROOT/'web/static'),name='static')
CFG=load_config(ROOT/'tracker_config.json')
SM=SessionManager(ROOT,CFG)

class StartRequest(BaseModel):
    sequence_id: str


class ExistingEdit(BaseModel):
    global_id: int
    dx: float=0.0
    dy: float=0.0
    dz: float=0.0
    dyaw_deg: float=0.0
    observation: str='normal'
    occlusion_side: str='none'
    severity: float=0.0

class NewCoilSpec(BaseModel):
    prototype: str='prototype_00'
    diameter: float=1.30
    length: float=1.10
    x: float
    y: float
    z: float=0.68
    yaw_deg: float=0.0
    layer: int=1
    observation: str='normal'
    occlusion_side: str='none'
    severity: float=0.0
    hint_dx: float=0.0
    hint_dy: float=0.0
    hint_dz: float=0.0

class CreateFrameRequest(BaseModel):
    existing: list[ExistingEdit]=[]
    new_coils: list[NewCoilSpec]=[]
    view_mode: str='MULTI_VIEW'
    second_layer_signal: bool|None=None
    seed: int|None=None

class SaveFailureRequest(BaseModel):
    note: str=''

@lru_cache(maxsize=256)
def _sample_cloud(file:str,max_per_instance:int=500):
    xyz,iid=load_algorithm_input(ROOT/file)
    out=[]
    # deterministic sampling: frame-specific seed without Python hash randomization
    seed=sum((i+1)*ord(c) for i,c in enumerate(file)) % (2**32-1)
    rng=np.random.default_rng(seed)
    for k in np.unique(iid):
        idx=np.flatnonzero(iid==k)
        if len(idx)>max_per_instance:
            idx=rng.choice(idx,max_per_instance,replace=False)
        p=xyz[idx]
        out.extend([[float(x),float(y),float(z),int(k)] for x,y,z in p])
    return out


def _frame_meta(file):
    try:
        r=next(x for x in SM.frame_manifest if x['file']==file)
        return {k:r.get(k,'') for k in ('file','sequence_id','frame_index','scene_name','num_objects_total','num_visible_objects','num_points','tags','description')}
    except StopIteration:
        return {'file':file}


def _public_state(sess,ev=None,ended=False):
    current_cloud=_sample_cloud(sess.current_file)
    previous_cloud=_sample_cloud(sess.previous_file) if sess.previous_file else []
    gaps=neighbor_gaps(sess.last_rows,CFG)
    seq=next(x for x in SM.sequence_index if x['sequence_id']==sess.sequence_id)
    return {
        'session_id':sess.session_id,
        'sequence':{
            'sequence_id':sess.sequence_id,'relation_mode':sess.relation_mode,
            'purpose':seq['purpose'],'num_frames':len(sess.frames),
            'track_mode':'continuous' if sess.relation_mode=='temporal_adjacent' else ('baseline_compare' if sess.relation_mode=='paired_to_baseline' else 'single_or_independent'),
        },
        'frame_index':sess.current_index,
        'frame_number':sess.current_index+1,
        'num_frames':len(sess.frames),
        'current_file':sess.current_file,
        'previous_file':sess.previous_file,
        'current_meta':_frame_meta(sess.current_file),
        'previous_meta':_frame_meta(sess.previous_file) if sess.previous_file else None,
        'at_last_frame':sess.current_index>=len(sess.frames)-1,
        'ended':ended,
        'can_create_next':sess.current_index>=len(sess.frames)-1,
        'is_interactive_frame':sess.current_file.startswith('USER_'),
        'current_cloud':current_cloud,
        'previous_cloud':previous_cloud,
        'objects':sess.last_rows,
        'previous_objects':sess.previous_rows,
        'neighbor_gaps':gaps,
        'stats':sess.stats,
        'debug':sess.last_debug,
        'frame_status':sess.last_debug.get('frame_status','OK'),
        'semantic_reason_codes':sess.last_debug.get('semantic_reason_codes',[]),
        # Only non-GT aggregate error count is omitted here. GT is fetched from /gt on demand.
    }

@app.get('/',response_class=HTMLResponse)
def index():
    return (ROOT/'web/index.html').read_text(encoding='utf-8')

@app.get('/api/sequences')
def sequences():
    return SM.sequences()

@app.post('/api/session/start')
def start(req:StartRequest):
    try:
        sess,ev=SM.start(req.sequence_id)
    except (KeyError,StopIteration):
        raise HTTPException(404,'sequence not found')
    except Exception as exc:
        raise HTTPException(400,str(exc))
    return _public_state(sess,ev)

@app.post('/api/session/{sid}/next')
def next_frame(sid:str):
    try:
        sess,ev,ended=SM.next(sid)
    except KeyError:
        raise HTTPException(404,'tracking session not found; please choose an initial frame again')
    return _public_state(sess,ev,ended)

@app.post('/api/session/{sid}/reset')
def reset(sid:str):
    SM.reset(sid)
    return {'ok':True}

@app.get('/api/session/{sid}/gt')
def gt(sid:str):
    """Evaluation-only endpoint. The matcher never calls current GT."""
    try: sess=SM.get(sid)
    except KeyError: raise HTTPException(404,'session not found')
    gt_rows=SM.current_gt_rows(sess)
    pred_by_iid={r['instance_id']:r for r in sess.last_rows if r['instance_id'] is not None}
    comparisons=[]; wrong=0; conservative=0
    for g in gt_rows:
        raw_iid=g.get('instance_id','')
        iid=None if raw_iid in ('',None) else int(float(raw_iid))
        gid=int(g['global_id']); gt_vis=g.get('visibility','VISIBLE')
        if iid is None:
            pred=next((r for r in sess.last_rows if r['global_id']==gid and r.get('visibility')=='UNOBSERVED'),None)
            comparisons.append({'instance_id':None,'gt_global_id':gid,'pred_global_id':pred['global_id'] if pred else None,'pred_state':pred.get('visibility') if pred else 'MISSING','gt_state':gt_vis,'result':'OK' if pred else 'MISS'})
            continue
        pred=pred_by_iid.get(iid); pred_gid=None if pred is None else pred['global_id']
        if sess.current_file in sess.editor_gt:
            mapped_gt=pred_gid
        else:
            mapped_gt=None if pred_gid is None else sess.id_map.get(int(pred_gid))
        if pred is None or pred_gid is None:
            result='CONSERVATIVE'; conservative+=1
        elif mapped_gt==gid:
            result='OK'
        else:
            result='WRONG_ID'; wrong+=1
        comparisons.append({'instance_id':iid,'gt_global_id':gid,'pred_global_id':pred_gid,'mapped_gt_global_id':mapped_gt,'pred_state':'MISSING' if pred is None else pred.get('visibility',pred['state']),'gt_state':'NEW' if int(g.get('is_new',0)) else gt_vis,'result':result})
    return {'file':sess.current_file,'comparisons':comparisons,'wrong_id_count':wrong,'conservative_count':conservative}

@app.get('/api/session/{sid}/editor')
def editor_seed(sid:str):
    try: return SM.editor_seed(sid)
    except KeyError: raise HTTPException(404,'session not found')

@app.post('/api/session/{sid}/preview-next')
def preview_next(sid:str, req:CreateFrameRequest):
    """Generate an exact point-cloud preview without committing tracker/session state.

    The same editor generator is used as create-next, but editor-only prototype bookkeeping
    is restored afterwards.  Therefore the preview is safe to call repeatedly while dragging.
    """
    try:
        sess=SM.get(sid)
    except KeyError:
        raise HTTPException(404,'session not found')
    model_backup=copy.deepcopy(sess.editor_models)
    try:
        created=SM.editor.create(sess,req.model_dump(),CFG)
    except Exception as exc:
        raise HTTPException(400,str(exc))
    finally:
        sess.editor_models=model_backup
    pts=[]
    rng=np.random.default_rng(int(created['seed'])+17)
    for o in created['scene']:
        p=np.asarray(o['points'],dtype=float)
        if len(p)>420:
            idx=rng.choice(len(p),420,replace=False); p=p[idx]
        gid=int(o['global_id'])
        pts.extend([[float(x),float(y),float(z),gid] for x,y,z in p])
    objs=[]
    for g in created['gt']:
        objs.append({
            'global_id':int(g['global_id']),'layer':int(g['layer']),'slot':int(g['slot']),
            'cluster':g.get('cluster'),'visibility':g.get('visibility','VISIBLE'),'is_new':int(g.get('is_new',0)),
            'center':[float(g['center_x']),float(g['center_y']),float(g['center_z'])],
            'yaw_deg':float(g['yaw_deg']),'diameter':float(g['diameter_nominal']),'radius':float(g['diameter_nominal'])/2.0,
            'length':float(g['length_nominal']),'point_count':int(g.get('point_count',0)),
        })
    return {'seed':created['seed'],'points':pts,'objects':objs,'warnings':created['warnings'],'view_mode':created.get('view_mode','MULTI_VIEW')}

@app.post('/api/session/{sid}/create-next')
def create_next(sid:str, req:CreateFrameRequest):
    try:
        sess,created,ev=SM.create_next(sid,req.model_dump())
    except KeyError:
        raise HTTPException(404,'session not found')
    except Exception as exc:
        raise HTTPException(400,str(exc))
    state=_public_state(sess,ev)
    state['editor_warnings']=created['warnings']; state['editor_seed']=created['seed']; state['interactive_eval']=ev
    return state

@app.post('/api/session/{sid}/save-failure')
def save_failure(sid:str, req:SaveFailureRequest):
    try: sess=SM.get(sid)
    except KeyError: raise HTTPException(404,'session not found')
    outdir=ROOT/'saved_failures'; outdir.mkdir(exist_ok=True)
    stamp=time.strftime('%Y%m%d_%H%M%S'); name=f'failure_{stamp}_{sid}_{sess.current_index:03d}.zip'; out=outdir/name
    prev=sess.previous_file; cur=sess.current_file
    def gt_for(file):
        if not file: return []
        if file in sess.editor_gt: return sess.editor_gt[file]
        return SM.ds.frame_rows(file)
    payloads={
        'user_note.txt':req.note,
        'edit_actions.json':sess.editor_actions.get(cur,[]),
        'current_scene_gt.json':gt_for(cur),
        'previous_scene_gt.json':gt_for(prev),
        'tracker_state_before.json':sess.last_tracker_state_before,
        'algorithm_result.json':sess.last_algorithm_result,
        'generation_seed.json':{'seed':sess.last_debug.get('editor_seed')},
        'robot_hints.json':sess.editor_hints.get(cur,[]),
        'tracker_config.json':CFG,
        'algorithm_version.json':{'version':'semantic_tracker_double_layer_v0.8','ui':'drag_scene_editor_v0.8','occlusion_rule':'OCCLUDED requires positive current instance evidence; zero points is abnormal/unobserved','identity_safety_rule':'UNCERTAIN/ABNORMAL preferred over wrong Global ID','motion_policy':'identity and abnormal motion are separated by normal/hard X envelopes','topdown_support':'TOP_DOWN_Z inferred from XYZ; center Z recovered from top surface minus radius'},
        'failure_summary.json':{'frame_status':sess.last_debug.get('frame_status','OK'),'semantic_reason_codes':sess.last_debug.get('semantic_reason_codes',[]),'history_count':len(sess.last_tracker_state_before),'current_object_rows':len(sess.last_rows),'uncertain_count':sum(r.get('state')=='UNCERTAIN' or r.get('visibility')=='UNCERTAIN' for r in sess.last_rows),'unobserved_count':sum(r.get('visibility')=='UNOBSERVED' for r in sess.last_rows),'stats':sess.stats},
        'session_context.json':{'session_id':sid,'sequence_id':sess.sequence_id,'previous_file':prev,'current_file':cur,'current_index':sess.current_index,'stats':sess.stats},
    }
    with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
        for arc,obj in payloads.items():
            if isinstance(obj,str): data=obj
            else: data=json.dumps(obj,ensure_ascii=False,indent=2)
            z.writestr(arc,data)
        if prev and (ROOT/prev).exists(): z.write(ROOT/prev,'previous_frame.ply')
        if cur and (ROOT/cur).exists(): z.write(ROOT/cur,'current_frame.ply')
    return {'ok':True,'filename':name,'download_url':f'/api/failures/{name}'}

@app.get('/api/failures/{name}')
def download_failure(name:str):
    p=(ROOT/'saved_failures'/Path(name).name)
    if not p.exists(): raise HTTPException(404,'failure package not found')
    return FileResponse(p,media_type='application/zip',filename=p.name)

@app.get('/api/health')
def health():
    return {'ok':True,'port_default':9999,'ui':'double_layer_drag_editor_v0_8','occlusion_rule':'positive-partial-evidence-required','identity_policy':'uncertain-over-wrong-id','topdown_z_supported':True,'motion_policy':'normal-and-hard-envelope; locked-layer1-foundation-when-layer2-active'}
