from pathlib import Path
import csv,json,sys,time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tracker import run_pair, evaluate_pair, load_config

def main():
    cfg=load_config(ROOT/'tracker_config.json')
    with open(ROOT/'labels/frame_pairs.csv','r',encoding='utf-8-sig',newline='') as f: pairs=list(csv.DictReader(f))
    selected=[p for p in pairs if p['recommended_for_id_eval']=='1']
    # priority + all recommended
    priority=['S10','S06','S07','S03','S11','S01','S02','S05','S08','S09']
    selected=sorted(selected,key=lambda p:(priority.index(p['sequence_id']) if p['sequence_id'] in priority else 99,p['pair_id']))
    rows=[]
    for p in selected:
        r=run_pair(ROOT,p['pair_id'],cfg,'confirmed_gt')
        e=evaluate_pair(ROOT,p['pair_id'],r)
        rows.append({
            'pair_id':p['pair_id'],'sequence_id':p['sequence_id'],'previous_file':p['previous_file'],'current_file':p['current_file'],
            'wrong_id':e['wrong_id'],'new_tp':e['new_tp'],'new_fp':e['new_fp'],'new_fn':e['new_fn'],
            'occ_tp':e['occ_tp'],'occ_fp':e['occ_fp'],'occ_fn':e['occ_fn'],
            'uncertain':sum(d['state']=='UNCERTAIN' for d in r['decisions']),'validator_errors':'|'.join(r['validator_errors']),
            'total_ms':round(r['timing_ms']['total_ms'],3),'pass_primary':e['pass_primary'] and not r['validator_errors']
        })
        print(rows[-1])
    out=ROOT/'results/regression_summary.csv'
    with out.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)
    summary={
      'pairs':len(rows),'wrong_id_total':sum(x['wrong_id'] for x in rows),'validator_failures':sum(bool(x['validator_errors']) for x in rows),
      'new_tp':sum(x['new_tp'] for x in rows),'new_fp':sum(x['new_fp'] for x in rows),'new_fn':sum(x['new_fn'] for x in rows),
      'occ_tp':sum(x['occ_tp'] for x in rows),'occ_fp':sum(x['occ_fp'] for x in rows),'occ_fn':sum(x['occ_fn'] for x in rows),
      'uncertain_total':sum(x['uncertain'] for x in rows),'max_total_ms':max(x['total_ms'] for x in rows)
    }
    (ROOT/'results/regression_summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')
    print('\nSUMMARY',json.dumps(summary,ensure_ascii=False))
    raise SystemExit(0 if summary['wrong_id_total']==0 and summary['validator_failures']==0 else 2)
if __name__=='__main__': main()
