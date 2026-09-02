from __future__ import annotations
import json,time
from pathlib import Path
import numpy as np
from .dataset_adapter import SyntheticDatasetAdapter
from .geometry import observe_frame
from .matching import build_candidates,select_anchors,select_semantic_rank_anchors,ordered_dp,decisions_from_actions

DEFAULT_CONFIG={
 'business':{'wrong_id_allowed':False,'allow_uncertain':True,'old_coil_removal_supported':False,'layer1_max_slots':10},
 'geometry':{'diameter_range_m':[1.1,1.5],'length_range_m':[0.5,1.7],'center_quantile':0.005,'size_quantile':0.01,'layer2_center_z_min_m':1.15},
 'motion_roi':{'x_absolute_max_m':2.0,'x_hard_max_m':2.40,'x_default_m':1.50,'y_default_m':0.40,'z_default_m':0.30,'y_absolute_max_m':0.40,'z_absolute_max_m':0.30},
 'matching':{
   'diameter_gate_m':0.36,'length_gate_m':0.38,'diameter_scale_m':0.20,'length_scale_m':0.25,'yaw_scale_deg':15.0,
   'w_position':0.62,'w_diameter':0.10,'w_length':0.10,'w_yaw':0.03,'w_quality':0.15,
   'anchor_quality_min':0.66,'anchor_best_cost_max':0.42,'anchor_margin_min':0.24,
   'skip_history_cost':0.90,'insert_current_cost':0.90
 },
 'new_coil':{'position_tolerance_m':0.50,'diameter_tolerance_m':0.25,'length_tolerance_m':0.30},
 'visualization':{'sample_points_per_instance':550},
 'topdown_view':{'mode':'TOP_DOWN_Z','ray_direction':[0,0,-1],'xy_cell_m':0.050,'surface_thickness_m':0.055,'min_visible_points_per_instance':80,'min_visible_ratio_per_instance':0.025},
 'topdown_detection':{'enabled':True,'z_span_quantile':0.01,'z_span_to_diameter_max':0.72,'z_span_to_diameter_strong':0.58,'top_z_quantile':0.995,'max_z_center_correction_m':0.55,'quality_factor':0.96},
 'semantic':{'uncertain_pair_cost':8.0,'relaxed_pair_penalty':0.25,'abnormal_motion_penalty':0.55,'abnormal_motion_confidence':0.64,'new_hint_weight':0.60,'hypothesis_margin_min':0.20},
}

def load_config(path=None):
    if path:
        with open(path,'r',encoding='utf-8') as f:return json.load(f)
    return json.loads(json.dumps(DEFAULT_CONFIG))

def _run_layer(layer,tracks,obs,accepted,anchors,cfg):
    lt=[t for t in tracks if t.layer==layer]; lo=[o for o in obs.values() if o.layer_guess==layer]
    la=[a for a in anchors if any(t.global_id==a[0] for t in lt)]
    return ordered_dp(lt,lo,accepted,cfg,la)

def global_validate(decisions,tracks,cfg):
    errors=[]
    gids=[d.global_id for d in decisions if d.state=='MATCHED' and d.global_id is not None]
    iids=[d.instance_id for d in decisions if d.instance_id is not None and d.state in ('MATCHED','NEW')]
    if len(gids)!=len(set(gids)): errors.append('DUPLICATE_GLOBAL_ID')
    if len(iids)!=len(set(iids)): errors.append('DUPLICATE_CURRENT_INSTANCE')
    # matched order monotonic by layer/cluster
    for layer in (1,2):
        ds=sorted([d for d in decisions if d.state=='MATCHED' and d.layer==layer],key=lambda d:d.slot)
        # x is checked earlier by ordered DP; this validates slot uniqueness.
        if len({d.slot for d in ds})!=len(ds): errors.append(f'LAYER{layer}_DUPLICATE_SLOT')
    old_l1={t.slot for t in tracks if t.layer==1}
    new_l1={d.slot for d in decisions if d.state=='NEW' and d.layer==1 and d.slot is not None}
    if len(old_l1|new_l1)>int(cfg['business']['layer1_max_slots']): errors.append('LAYER1_CAPACITY_EXCEEDED')
    return errors

def run_pair(dataset_root,pair_id,cfg=None,history_mode='confirmed_gt'):
    cfg=cfg or load_config(); ds=SyntheticDatasetAdapter(dataset_root); pair=ds.pair(pair_id)
    timing={}; t0=time.perf_counter()
    history=ds.bootstrap_history(pair['previous_file'],cfg,estimated_centers=(history_mode=='estimated_geometry'))
    timing['history_ms']=(time.perf_counter()-t0)*1000
    t=time.perf_counter(); inst=ds.load_instances(pair['current_file']); timing['load_current_ms']=(time.perf_counter()-t)*1000
    t=time.perf_counter(); obs=observe_frame(inst,cfg); timing['geometry_ms']=(time.perf_counter()-t)*1000
    t=time.perf_counter(); candidates,accepted=build_candidates(history,obs,cfg); timing['candidate_ms']=(time.perf_counter()-t)*1000
    t=time.perf_counter(); hints=ds.robot_hints(pair['current_file']); semantic_anchors,semantic_anchor_audit=select_semantic_rank_anchors(history,obs,accepted,hints,cfg)
    if semantic_anchors:
        anchors=semantic_anchors; anchor_audit=[{'mode':'SEMANTIC_RANK_LOCK',**x} for x in semantic_anchor_audit]
    else:
        anchors,anchor_audit=select_anchors(history,obs,accepted,cfg); anchor_audit=[{'mode':'GEOMETRIC_ANCHOR',**x} for x in anchor_audit]+[{'mode':'SEMANTIC_RANK_AUDIT',**x} for x in semantic_anchor_audit]
    timing['anchor_ms']=(time.perf_counter()-t)*1000
    t=time.perf_counter(); actions=[]; layer_debug={}; total_cost=0
    layers=sorted(set([x.layer for x in history]+[o.layer_guess for o in obs.values()]))
    for layer in layers:
        aa,cost,dbg=_run_layer(layer,history,obs,accepted,anchors,cfg); actions.extend(aa); total_cost+=cost; layer_debug[str(layer)]={'cost':cost,**dbg,'actions':aa}
    timing['dp_ms']=(time.perf_counter()-t)*1000
    t=time.perf_counter(); decisions=decisions_from_actions(actions,history,obs,hints,cfg,accepted); errors=global_validate(decisions,history,cfg); timing['decision_ms']=(time.perf_counter()-t)*1000
    timing['total_ms']=(time.perf_counter()-t0)*1000
    return {
      'pair':pair,'history_mode':history_mode,'config':cfg,'history':[x.json() for x in history],
      'observations':[obs[k].json() for k in sorted(obs)],'hints':[h.json() for h in hints],
      'candidates':[c.json() for c in candidates],'anchors':[{'global_id':g,'instance_id':i,'cost':c} for g,i,c in anchors],
      'anchor_audit':anchor_audit,'dp':layer_debug,'actions':actions,'decisions':[d.json() for d in decisions],
      'validator_errors':errors,'timing_ms':timing,'total_dp_cost':total_cost
    }
