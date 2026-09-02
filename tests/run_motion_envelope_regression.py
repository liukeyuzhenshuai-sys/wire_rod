from pathlib import Path
import json, zipfile, tempfile, sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tracker.models import TrackState, NewCoilHint
from tracker.ply import split_instances
from tracker.session import match_from_tracks, commit_step, movement_rows

CASE=ROOT/'regression'/'failures'/'failure_20260818_163534_90ea5def0107_004.zip'

def load_json(p): return json.loads(Path(p).read_text(encoding='utf-8'))

def run_case(tracks,hints,instances,cfg):
    result=match_from_tracks(tracks,instances,hints,cfg)
    new_tracks,decisions=commit_step(tracks,result,cfg)
    rows=movement_rows(tracks,new_tracks,decisions,result['observations'])
    return result,rows

def main():
    cfg=load_json(ROOT/'tracker_config.json')
    with tempfile.TemporaryDirectory(prefix='coil_v07_motion_') as td:
        td=Path(td)
        with zipfile.ZipFile(CASE,'r') as z:z.extractall(td)
        tracks=[]
        for d in load_json(td/'tracker_state_before.json'):
            d=d.copy();d['center']=np.asarray(d['center'],dtype=float);tracks.append(TrackState(**d))
        hints=[]
        for d in load_json(td/'robot_hints.json'):
            d=d.copy();d['target_center']=np.asarray(d['target_center'],dtype=float);hints.append(NewCoilHint(**d))
        base=split_instances(td/'current_frame.ply')
        result_a,rows_a=run_case(tracks,hints,{k:v.copy() for k,v in base.items()},cfg)
        g5a=next(r for r in rows_a if r.get('global_id')==5)
        assert g5a['state']=='MATCHED' and result_a['frame_status']=='ABNORMAL'
        assert 'ABNORMAL_X_MOTION_SEMANTIC_RESCUE' in (g5a.get('reason') or '')

        shifted={k:v.copy() for k,v in base.items()}
        shifted[292][:,0]+=0.30  # pushes true ID5 beyond x_hard_max_m=2.4 while preserving order
        result_b,rows_b=run_case(tracks,hints,shifted,cfg)
        g5b=next(r for r in rows_b if r.get('global_id')==5)
        assert g5b['state']=='UNCERTAIN',g5b
        assert 'OUTSIDE_X_HARD_ROI' in (g5b.get('reason') or ''),g5b
        # Hard-envelope failure must not cause suffix ID shift.
        for gid in range(5):
            r=next(x for x in rows_b if x.get('global_id')==gid)
            assert r['state']=='MATCHED',(gid,r)
        out={
            'version':'v0.8',
            'normal_x_max_m':cfg['motion_roi']['x_absolute_max_m'],
            'hard_x_max_m':cfg['motion_roi']['x_hard_max_m'],
            'inside_hard_envelope':{'frame_status':result_a['frame_status'],'gid5':{k:g5a.get(k) for k in ('state','delta_xyz','reason','confidence')}},
            'beyond_hard_envelope':{'frame_status':result_b['frame_status'],'gid5':{k:g5b.get(k) for k in ('state','delta_xyz','reason','confidence')}},
        }
        (ROOT/'results'/'motion_envelope_regression.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(out,ensure_ascii=False,indent=2))
        print('PASS: identity and motion abnormality are decoupled inside hard envelope; hard envelope still protects identity precision.')

if __name__=='__main__':main()
