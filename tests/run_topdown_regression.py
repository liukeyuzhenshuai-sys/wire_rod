from pathlib import Path
import json, sys, time, csv
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tracker.session import SessionManager
from tracker.engine import load_config


def main():
    cfg=load_config(ROOT/'tracker_config.json')
    sm=SessionManager(ROOT,cfg)
    seqs=[s for s in sm.sequences() if s['sequence_id'].startswith('Z')]
    object_rows=list(csv.DictReader(open(ROOT/'labels'/'object_gt.csv',encoding='utf-8-sig')))
    zero_point_gt=[{'file':r['file'],'global_id':r['global_id'],'visibility':r['visibility']} for r in object_rows if r['sequence_id'].startswith('Z') and int(float(r.get('point_count') or 0))<=0]
    summary=[]
    for s in seqs:
        sess,ev=sm.start(s['sequence_id'])
        frames=[{'frame':sess.current_file,'wrong_id':ev['wrong_id_count'],'frame_status':'INITIAL','uncertain':0}]
        t0=time.perf_counter()
        while sess.current_index < len(sess.frames)-1:
            sess,ev,_=sm.next(sess.session_id)
            frames.append({
                'frame':sess.current_file,
                'wrong_id':ev['wrong_id_count'],
                'frame_status':sess.last_debug.get('frame_status','OK'),
                'uncertain':sum(r['state']=='UNCERTAIN' for r in sess.last_rows),
                'semantic_reason_codes':sess.last_debug.get('semantic_reason_codes',[]),
                'topdown_layer_assignment':sess.last_debug.get('topdown_layer_assignment',{}),
                'observations':[{'instance_id':r.get('instance_id'),'global_id':r.get('global_id'),'observation_mode':r.get('observation_mode'),'state':r.get('state')} for r in sess.last_rows if r.get('instance_id') is not None],
            })
        elapsed=(time.perf_counter()-t0)*1000
        rec={'sequence_id':s['sequence_id'],'num_frames':len(sess.frames),'wrong_id_total':sess.stats['wrong_id'],'uncertain_total':sess.stats['uncertain'],'elapsed_ms':elapsed,'frames':frames}
        summary.append(rec)
        print(f"{s['sequence_id']}: frames={len(sess.frames)} wrong={rec['wrong_id_total']} uncertain={rec['uncertain_total']} elapsed_ms={elapsed:.1f}")
    out={
        'version':'v0.8',
        'view_model':'camera above, rays along world -Z, global XY z-buffer; not z>center crop',
        'zero_point_gt_objects':zero_point_gt,
        'total_sequences':len(summary),
        'total_frames':sum(x['num_frames'] for x in summary),
        'total_wrong_id':sum(x['wrong_id_total'] for x in summary),
        'total_uncertain':sum(x['uncertain_total'] for x in summary),
        'sequences':summary,
    }
    (ROOT/'results'/'topdown_sequential_regression.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
    print(f"TOTAL: sequences={out['total_sequences']} frames={out['total_frames']} wrong={out['total_wrong_id']} uncertain={out['total_uncertain']} zero_point_gt={len(zero_point_gt)}")
    assert not zero_point_gt, f'Z dataset violates positive-observation rule: {zero_point_gt[:3]}'
    raise SystemExit(0 if out['total_wrong_id']==0 else 2)

if __name__=='__main__': main()
