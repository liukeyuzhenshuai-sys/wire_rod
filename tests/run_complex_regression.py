from pathlib import Path
import json, sys, time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tracker.session import SessionManager
from tracker.engine import load_config

def main():
    cfg=load_config(ROOT/'tracker_config.json')
    sm=SessionManager(ROOT,cfg)
    seqs=[s for s in sm.sequences() if s['sequence_id'].startswith('C')]
    summary=[]
    for s in seqs:
        sess,ev=sm.start(s['sequence_id'])
        frames=[{'frame':sess.current_file,'wrong_id':ev['wrong_id_count'],'eval':ev}]
        t0=time.perf_counter()
        while sess.current_index < len(sess.frames)-1:
            sess,ev,ended=sm.next(sess.session_id)
            frames.append({'frame':sess.current_file,'wrong_id':ev['wrong_id_count'],'eval':ev,'debug':sess.last_debug})
        elapsed=(time.perf_counter()-t0)*1000
        rec={'sequence_id':s['sequence_id'],'num_frames':len(sess.frames),'wrong_id_total':sess.stats['wrong_id'],
             'new_pred':sess.stats['new'],'occluded_pred':sess.stats['occluded'],'uncertain_pred':sess.stats['uncertain'],
             'elapsed_ms':elapsed,'frames':frames}
        summary.append(rec)
        print(f"{s['sequence_id']}: frames={len(sess.frames)} wrong={sess.stats['wrong_id']} new={sess.stats['new']} occ={sess.stats['occluded']} uncertain={sess.stats['uncertain']} elapsed_ms={elapsed:.1f}")
    out={'total_sequences':len(summary),'total_frames':sum(x['num_frames'] for x in summary),
         'total_wrong_id':sum(x['wrong_id_total'] for x in summary),'sequences':summary}
    (ROOT/'results'/'complex_sequential_regression.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
    print(f"TOTAL: sequences={out['total_sequences']} frames={out['total_frames']} wrong_id={out['total_wrong_id']}")
    raise SystemExit(0 if out['total_wrong_id']==0 else 2)

if __name__=='__main__':
    main()
