from pathlib import Path
import json, zipfile, tempfile, sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tracker.models import TrackState, NewCoilHint
from tracker.ply import split_instances
from tracker.session import match_from_tracks, commit_step, movement_rows

CASE=ROOT/'regression'/'failures'/'failure_20260818_163534_90ea5def0107_004.zip'

def load_json(p):
    return json.loads(Path(p).read_text(encoding='utf-8'))

def main():
    cfg=load_json(ROOT/'tracker_config.json')
    with tempfile.TemporaryDirectory(prefix='coil_v07_case_') as td:
        td=Path(td)
        with zipfile.ZipFile(CASE,'r') as z: z.extractall(td)
        tracks=[]
        for d in load_json(td/'tracker_state_before.json'):
            d=d.copy(); d['center']=np.asarray(d['center'],dtype=float); tracks.append(TrackState(**d))
        hints=[]
        for d in load_json(td/'robot_hints.json'):
            d=d.copy(); d['target_center']=np.asarray(d['target_center'],dtype=float); hints.append(NewCoilHint(**d))
        inst=split_instances(td/'current_frame.ply')
        result=match_from_tracks(tracks,inst,hints,cfg)
        new_tracks,decisions=commit_step(tracks,result,cfg)
        rows=movement_rows(tracks,new_tracks,decisions,result['observations'])
        gt={int(x['instance_id']):int(x['global_id']) for x in load_json(td/'current_scene_gt.json')}
        wrong=[]
        for r in rows:
            iid=r.get('instance_id'); gid=r.get('global_id')
            if iid is None or gid is None or r.get('state')=='UNCERTAIN':
                continue
            if gt.get(int(iid)) != int(gid):
                wrong.append({'pred_gid':gid,'instance_id':iid,'gt_gid':gt.get(int(iid))})
        summary={
            'case':'failure_20260818_163534_90ea5def0107_004',
            'frame_status':result.get('frame_status'),
            'semantic_reason_codes':result.get('semantic_reason_codes',[]),
            'wrong_committed':wrong,
            'rows':[{k:r.get(k) for k in ('global_id','instance_id','state','visibility','reason')} for r in rows],
        }
        out=ROOT/'results'/'failure_case_90ea_v0_8.json'
        out.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(summary,ensure_ascii=False,indent=2))
        by_gid={r.get('global_id'):r for r in rows if r.get('global_id') is not None}
        assert not wrong, f'wrong committed IDs: {wrong}'
        for gid in range(5):
            assert by_gid[gid]['state']=='MATCHED', (gid,by_gid[gid])
            assert gt[int(by_gid[gid]['instance_id'])]==gid, (gid,by_gid[gid])
        assert by_gid[5]['state']=='MATCHED', by_gid[5]
        assert 'ABNORMAL_X_MOTION_SEMANTIC_RESCUE' in (by_gid[5].get('reason') or ''), by_gid[5]
        assert result.get('frame_status')=='ABNORMAL', result.get('frame_status')
        assert by_gid[5]['instance_id'] is not None, 'ID5 must retain positive current evidence in UI result'
        assert by_gid[6]['state']=='NEW', by_gid[6]
        print('PASS: semantic guard prevents chain ID shift; ID5 identity is preserved while X motion is explicitly ABNORMAL inside the hard envelope.')

if __name__=='__main__':
    main()
