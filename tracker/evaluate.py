from __future__ import annotations
from .dataset_adapter import SyntheticDatasetAdapter

def evaluate_pair(dataset_root,pair_id,result):
    ds=SyntheticDatasetAdapter(dataset_root); gt=ds.gt_transition(pair_id)
    gt_by_iid={int(float(r['current_instance_id'])):r for r in gt if r['current_instance_id'] not in ('','-1')}
    gt_missing={int(r['global_id']):r for r in gt if r['current_instance_id'] in ('','-1')}
    decisions=result['decisions']; rows=[]; wrong=0; matched_eval=0; new_tp=new_fp=new_fn=0; occ_tp=occ_fp=occ_fn=0
    # exact ID correctness only applies to MATCHED old objects. NEW IDs are arbitrary at birth.
    for d in decisions:
        iid=d['instance_id']; gid=d['global_id']; state=d['state']
        if iid is not None:
            g=gt_by_iid.get(int(iid)); status='NO_GT'
            if g:
                isnew=int(g['is_new_gt'])==1
                if state=='MATCHED':
                    matched_eval+=1
                    ok=(not isnew and int(g['global_id'])==int(gid)); status='OK' if ok else 'WRONG_ID'; wrong+=0 if ok else 1
                elif state=='NEW':
                    status='NEW_OK' if isnew else 'NEW_FALSE'; new_tp+=1 if isnew else 0; new_fp+=0 if isnew else 1
                elif state=='UNCERTAIN': status='UNCERTAIN'
            rows.append({**d,'gt_global_id':None if not g else int(g['global_id']),'gt_is_new':None if not g else int(g['is_new_gt']),'eval':status})
    gt_new=sum(int(r['is_new_gt']) for r in gt); new_fn=max(0,gt_new-new_tp)
    pred_occ={int(d['global_id']) for d in decisions if d['state'] in ('OCCLUDED','UNOBSERVED') and d['global_id'] is not None}  # legacy zero-point GT is treated as UNOBSERVED in V0.4
    gt_occ=set(gt_missing)
    occ_tp=len(pred_occ&gt_occ); occ_fp=len(pred_occ-gt_occ); occ_fn=len(gt_occ-pred_occ)
    return {'wrong_id':wrong,'matched_evaluated':matched_eval,'new_tp':new_tp,'new_fp':new_fp,'new_fn':new_fn,'occ_tp':occ_tp,'occ_fp':occ_fp,'occ_fn':occ_fn,'rows':rows,'gt_transition':gt,'pass_primary':wrong==0}
