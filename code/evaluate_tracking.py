"""Conservative tracking evaluator for V3.1.

Prediction CSV columns:
    pair_id,current_instance_id,pred_global_id,pred_state,confidence

States:
- MATCHED: current_instance_id >= 0, predicted historical global ID
- NEW: current_instance_id >= 0, predicted new global ID
- UNCERTAIN: current_instance_id >= 0, pred_global_id may be empty
- OCCLUDED: current_instance_id = -1, pred_global_id = historical global ID

Business priority: WRONG_ID must be zero. UNCERTAIN is allowed.
"""
from pathlib import Path
import csv, sys

ROOT=Path(__file__).resolve().parents[1]
GT_PATH=ROOT/'labels'/'transition_gt.csv'

def read_rows(path):
    with open(path,'r',encoding='utf-8-sig',newline='') as f:
        return list(csv.DictReader(f))

def to_i(x,default=None):
    try:
        if x is None or x=='': return default
        return int(float(x))
    except Exception:
        return default

def main(pred_path):
    pred=read_rows(pred_path)
    if not pred:
        raise SystemExit('prediction file is empty')
    pair_ids={p['pair_id'] for p in pred}
    gt=[g for g in read_rows(GT_PATH) if g['pair_id'] in pair_ids]

    gt_by_inst={}
    new_gt_inst=set()
    occ_gt=set()
    visible_gt_instances=set()
    for g in gt:
        pair=g['pair_id']; iid=to_i(g['current_instance_id'],-1); gid=to_i(g['global_id'])
        if iid>=0:
            gt_by_inst[(pair,iid)]=gid
            visible_gt_instances.add((pair,iid))
            if g['relation_gt']=='NEW': new_gt_inst.add((pair,iid))
        if g['current_visibility']=='OCCLUDED':
            occ_gt.add((pair,gid))

    assigned=correct=wrong=uncertain=0
    predicted_instances=set()
    new_pred_inst=set()
    occ_pred=set()
    for p in pred:
        state=p['pred_state'].strip().upper(); pair=p['pair_id']
        iid=to_i(p['current_instance_id'],-1); gid=to_i(p['pred_global_id'])
        if iid>=0: predicted_instances.add((pair,iid))
        if state=='UNCERTAIN' or (gid is None and state!='OCCLUDED'):
            uncertain+=1
            continue
        if state=='OCCLUDED':
            if gid is not None: occ_pred.add((pair,gid))
            continue
        if iid<0 or gid is None:
            continue
        assigned+=1
        truth=gt_by_inst.get((pair,iid))
        if truth==gid: correct+=1
        else: wrong+=1
        if state=='NEW': new_pred_inst.add((pair,iid))

    precision=correct/assigned if assigned else 0.0
    coverage=assigned/len(visible_gt_instances) if visible_gt_instances else 0.0
    new_tp=len(new_pred_inst & new_gt_inst); new_fp=len(new_pred_inst-new_gt_inst); new_fn=len(new_gt_inst-new_pred_inst)
    new_prec=new_tp/(new_tp+new_fp) if new_tp+new_fp else (1.0 if not new_gt_inst else 0.0)
    new_rec=new_tp/(new_tp+new_fn) if new_tp+new_fn else 1.0
    occ_tp=len(occ_pred & occ_gt); occ_fp=len(occ_pred-occ_gt); occ_fn=len(occ_gt-occ_pred)

    print(f'pairs_evaluated={len(pair_ids)} visible_gt_instances={len(visible_gt_instances)}')
    print(f'assigned={assigned} correct={correct} WRONG_ID={wrong} uncertain_rows={uncertain}')
    print(f'ID precision among assigned = {precision:.6f}')
    print(f'assignment coverage = {coverage:.6f}')
    print(f'NEW precision={new_prec:.6f} recall={new_rec:.6f} TP={new_tp} FP={new_fp} FN={new_fn}')
    print(f'OCCLUDED TP={occ_tp} FP={occ_fp} FN={occ_fn}')
    print('PRIMARY PASS' if wrong==0 else 'PRIMARY FAIL: wrong ID exists')

if __name__=='__main__':
    if len(sys.argv)!=2:
        print('usage: python code/evaluate_tracking.py predictions.csv')
        raise SystemExit(2)
    main(sys.argv[1])
