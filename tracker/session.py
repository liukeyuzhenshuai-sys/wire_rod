from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
import copy
import csv
import math
import time
import uuid
import numpy as np

from .dataset_adapter import SyntheticDatasetAdapter
from .engine import global_validate
from .geometry import observe_frame
from .matching import build_candidates, select_anchors, select_semantic_rank_anchors, ordered_dp, decisions_from_actions, semantic_constrained_match, apply_topdown_semantic_layer_assignment
from .models import TrackState, Decision, Observation
from .editor import InteractiveSceneEditor


def _clone_track(t: TrackState) -> TrackState:
    return TrackState(
        global_id=t.global_id, layer=t.layer, slot=t.slot, cluster=t.cluster,
        stable_diameter=t.stable_diameter, stable_length=t.stable_length,
        center=np.array(t.center, dtype=float).copy(), yaw_deg=t.yaw_deg,
        visibility=t.visibility, last_instance_id=t.last_instance_id,
        confidence=t.confidence, reference_point_count=t.reference_point_count,
    )


def clone_tracks(tracks):
    return [_clone_track(t) for t in tracks]


def _axis_from_yaw(yaw_deg: float):
    r=math.radians(float(yaw_deg))
    return [math.sin(r), math.cos(r), 0.0]


def _infer_initial_semantics(observations: dict[int, Observation], cfg: dict):
    """Assign initial Layer/Slot/Cluster and deterministic global IDs from geometry.

    This deliberately does not read object_gt/global_id/slot.  The rule is the
    production semantic model we agreed on: L1 has at most 10 slots, occupied
    objects can form a left edge cluster, a right edge cluster, or both.  L2 is
    ordered independently and uses global IDs 10+slot in this V1 demo.
    """
    cap=int(cfg['business']['layer1_max_slots'])
    icfg=cfg.get('initialization',{})
    split_gap=float(icfg.get('cluster_split_gap_m',2.20))
    right_x=float(icfg.get('right_only_min_mean_x_m',9.0))

    assigned=[]
    l1=sorted([o for o in observations.values() if o.layer_guess==1], key=lambda o:o.center[0])
    l2=sorted([o for o in observations.values() if o.layer_guess==2], key=lambda o:o.center[0])

    if l1:
        if len(l1)>cap:
            raise ValueError(f'initial L1 contains {len(l1)} objects > capacity {cap}')
        if len(l1)==cap:
            groups=[('full_L1', list(range(cap)), l1)]
        else:
            gaps=np.diff([float(o.center[0]) for o in l1]) if len(l1)>=2 else np.array([])
            if len(gaps) and float(np.max(gaps))>=split_gap:
                k=int(np.argmax(gaps))+1
                lo=l1[:k]; ro=l1[k:]
                groups=[]
                if lo: groups.append(('left',list(range(len(lo))),lo))
                if ro: groups.append(('right',list(range(cap-len(ro),cap)),ro))
            else:
                mean_x=float(np.mean([o.center[0] for o in l1]))
                if mean_x>=right_x:
                    groups=[('right',list(range(cap-len(l1),cap)),l1)]
                else:
                    groups=[('left',list(range(len(l1))),l1)]
        for cluster,slots,objs in groups:
            for slot,o in zip(slots,objs):
                assigned.append((o,1,int(slot),cluster,int(slot)))

    for slot,o in enumerate(l2):
        assigned.append((o,2,slot,'layer2',10+slot))
    return assigned



def _infer_double_layer_initial_semantics(observations: dict[int, Observation], cfg: dict):
    """Bootstrap an already double-layer scene using only XYZ-derived observations.

    The production signal tells us second-layer loading is active. Business semantics
    then imply exactly 10 positively observed foundation coils. Remaining observations
    are upper coils. Upper slot k is the support valley between lower slots k and k+1.
    """
    import itertools
    dc=cfg.get('double_layer',{}); req=int(dc.get('bootstrap_layer1_count',10))
    obs=list(observations.values()); n=len(obs); upper_n=n-req
    audit={'mode':'DOUBLE_LAYER_BOOTSTRAP','observation_count':n,'required_layer1':req,'upper_count':upper_n,'status':'OK'}
    if upper_n<0:
        audit.update({'status':'ABNORMAL','reason_code':'ABNORMAL_BOOTSTRAP_L1_NOT_FULL','message':f'only {n} positive instances but {req} first-layer coils are required'})
        return None,audit
    max_upper=int(cfg.get('business',{}).get('layer2_max_coils',8))
    if upper_n>max_upper:
        audit.update({'status':'ABNORMAL','reason_code':'ABNORMAL_BOOTSTRAP_LAYER2_CAPACITY','message':f'{upper_n} upper observations exceed configured maximum {max_upper}'})
        return None,audit
    # The upper top surface is structurally higher. This remains valid for TOP_DOWN_Z
    # where lower bboxes may be clipped, because robust top_z is retained explicitly.
    ranked=sorted(obs,key=lambda o:float(getattr(o,'top_z',o.bbox_max[2])),reverse=True)
    upper_ids={o.instance_id for o in ranked[:upper_n]}
    l1=sorted([o for o in obs if o.instance_id not in upper_ids],key=lambda o:float(o.center[0]))
    l2=sorted([o for o in obs if o.instance_id in upper_ids],key=lambda o:float(o.center[0]))
    if len(l1)!=req:
        audit.update({'status':'ABNORMAL','reason_code':'ABNORMAL_BOOTSTRAP_LAYER_SPLIT','message':'could not resolve exactly 10 foundation observations'})
        return None,audit
    for o in l1:o.layer_guess=1
    for o in l2:o.layer_guess=2

    # Full-L1 semantics are strong enough to reconstruct a heavily top-clipped
    # foundation lattice.  In an already double-layer initial frame, the lower
    # coils may expose only narrow top strips, so their per-instance visible width
    # is not a safe diameter estimate.  Ten contiguous slots give us a much more
    # stable geometric cue: robust center-to-center spacing.  Use it only when it
    # lies inside the configured physical diameter range.
    lattice_audit={}
    if len(l1)==req and req>=3:
        xs=np.asarray([float(o.center[0]) for o in l1],dtype=float)
        dif=np.diff(xs)
        spacing=float(np.median(dif)) if len(dif) else 0.0
        dmin,dmax=[float(x) for x in cfg.get('geometry',{}).get('diameter_range_m',[1.1,1.5])]
        if dmin*0.92 <= spacing <= dmax*1.08:
            intercept=float(np.median(xs-np.arange(req,dtype=float)*spacing))
            fit=intercept+np.arange(req,dtype=float)*spacing
            residual=xs-fit
            lattice_audit={'spacing_m':spacing,'intercept_m':intercept,'max_abs_x_residual_m':float(np.max(np.abs(residual))),'applied':True}
            for slot,o in enumerate(l1):
                if getattr(o,'observation_mode','MULTI_VIEW')=='TOP_DOWN_Z':
                    o.center[0]=float(fit[slot])
                    o.diameter=float(np.clip(spacing,dmin,dmax))
                    o.center[2]=float(getattr(o,'top_z',o.center[2]+o.diameter/2.0)-o.diameter/2.0)
                    o.center_method='DOUBLE_LAYER_BOOTSTRAP_L1_LATTICE'
        else:
            lattice_audit={'spacing_m':spacing,'applied':False,'reason':'spacing_outside_physical_diameter_range'}
    audit['foundation_lattice']=lattice_audit

    assigned=[]; foundation=[]
    for slot,o in enumerate(l1):
        assigned.append((o,1,slot,'full_L1',slot)); foundation.append((slot,o))
    # Build the 9 discrete support valleys from the reconstructed/observed foundation order.
    support={k:(np.asarray(l1[k].center,dtype=float)+np.asarray(l1[k+1].center,dtype=float))/2.0 for k in range(req-1)}
    if upper_n:
        best=None
        # Support order cannot reverse, so choose an increasing subset of valleys.
        for slots in itertools.combinations(range(req-1),upper_n):
            score=0.0
            for o,k in zip(l2,slots):
                score+=float(np.linalg.norm(np.asarray(o.center[:2])-np.asarray(support[k][:2])))
            if best is None or score<best[0]:best=(score,slots)
        if best is None:
            audit.update({'status':'ABNORMAL','reason_code':'ABNORMAL_BOOTSTRAP_SUPPORT_ASSIGNMENT','message':'no legal support-valley assignment'})
            return None,audit
        normal=float(dc.get('support_slot_max_center_error_m',0.42)); hard=float(dc.get('support_slot_hard_center_error_m',0.65))
        support_rows=[]
        for o,k in zip(l2,best[1]):
            err=float(np.linalg.norm(np.asarray(o.center[:2])-np.asarray(support[k][:2])))
            support_rows.append({'instance_id':int(o.instance_id),'slot':int(k),'supported_by':[int(k),int(k+1)],'xy_error_m':err})
            if err>hard:
                audit.update({'status':'ABNORMAL','reason_code':'ABNORMAL_BOOTSTRAP_UPPER_OFF_SUPPORT','support_assignment':support_rows,'message':f'upper instance {o.instance_id} is {err:.3f}m from nearest legal support valley'})
                return None,audit
            assigned.append((o,2,int(k),'layer2',10+int(k)))
        audit['support_assignment']=support_rows
        audit['support_score']=float(best[0])
        audit['support_normal_tolerance_m']=normal; audit['support_hard_tolerance_m']=hard
    audit['foundation_instance_order']=[int(o.instance_id) for o in l1]
    return assigned,audit

def initialize_tracks(observations: dict[int, Observation], cfg: dict, second_layer_signal: bool=False):
    tracks=[]
    decisions=[]
    bootstrap_audit={'mode':'SINGLE_LAYER_BOOTSTRAP','status':'OK'}
    assigned=None
    if second_layer_signal and bool(cfg.get('double_layer',{}).get('enabled',False)):
        assigned,bootstrap_audit=_infer_double_layer_initial_semantics(observations,cfg)
    if assigned is None:
        assigned=_infer_initial_semantics(observations,cfg)
    reason_prefix=('double-layer bootstrap: full L1 foundation + support-valley topology; ' if bootstrap_audit.get('status')=='OK' and bootstrap_audit.get('mode')=='DOUBLE_LAYER_BOOTSTRAP' else
                   ((bootstrap_audit.get('reason_code','ABNORMAL_DOUBLE_LAYER_BOOTSTRAP')+': fallback generic initialization; ') if second_layer_signal else ''))
    for o,layer,slot,cluster,gid in assigned:
        tracks.append(TrackState(
            global_id=gid, layer=layer, slot=slot, cluster=cluster,
            stable_diameter=float(o.diameter), stable_length=float(o.length),
            center=np.array(o.center,dtype=float), yaw_deg=float(o.yaw_deg),
            visibility=('VIEW_PARTIAL' if getattr(o,'observation_mode','MULTI_VIEW')=='TOP_DOWN_Z' else 'VISIBLE'), last_instance_id=o.instance_id,
            confidence=float(o.quality), reference_point_count=int(o.point_count),
        ))
        decisions.append(Decision(
            instance_id=o.instance_id, global_id=gid, state='INITIAL',
            confidence=float(o.quality), layer=layer, slot=slot, cluster=cluster,
            reason=reason_prefix+'initial semantic assignment from geometry/order; no current GT identity used'
        ))
    return tracks, decisions


def _run_layer(layer,tracks,obs,accepted,anchors,cfg):
    lt=[t for t in tracks if t.layer==layer]
    lo=[o for o in obs.values() if o.layer_guess==layer]
    la=[a for a in anchors if any(t.global_id==a[0] for t in lt)]
    return ordered_dp(lt,lo,accepted,cfg,la)


def match_from_tracks(tracks, instances, hints, cfg, second_layer_signal: bool=False):
    """Run one semantic-first matcher step from committed persistent TrackState.

    V0.6 deliberately does not let geometric anchors or SKIP_HISTORY define identity.
    Candidate/anchor calculations are retained for diagnostics, while the production
    decision path is semantic_constrained_match: expected NEW cardinality + layer/order
    topology first, positive current observation for every old coil, geometry second.
    """
    t0=time.perf_counter()
    local_cfg=copy.deepcopy(cfg)
    local_cfg['_runtime']={'second_layer_signal':bool(second_layer_signal)}
    obs=observe_frame(instances,local_cfg)
    # TOP_DOWN_Z can bias a strongly clipped lower-layer bbox upward. Resolve layer
    # membership from immutable historical-layer cardinality + robot NEW hints before
    # any candidate layer gate is applied.
    layer_assignment=apply_topdown_semantic_layer_assignment(tracks,obs,hints,local_cfg)
    t_geom=time.perf_counter()
    candidates,accepted=build_candidates(tracks,obs,local_cfg)
    t_cand=time.perf_counter()

    # Geometric anchors are diagnostic only in V0.6.  They must never force a global ID
    # when the semantic configuration would become inconsistent.
    diag_anchors,diag_anchor_audit=select_anchors(tracks,obs,accepted,local_cfg)
    t_anchor=time.perf_counter()
    sem=semantic_constrained_match(tracks,obs,hints,candidates,local_cfg)
    t_sem=time.perf_counter()
    decisions=sem['decisions']
    errors=global_validate(decisions,tracks,local_cfg)
    # Semantic abnormalities are intentionally surfaced, not hidden as validator bugs.
    frame_status=sem['frame_status']
    reason_codes=list(sem['reason_codes'])
    if errors and frame_status=='OK':
        frame_status='ABNORMAL'
    t_end=time.perf_counter()
    return {
        'observations':obs,
        'topdown_layer_assignment':layer_assignment,
        'candidates':candidates,
        'anchors':[],
        'anchor_audit':[{'mode':'DIAGNOSTIC_ONLY',**x} for x in diag_anchor_audit],
        'diagnostic_anchors':diag_anchors,
        'actions':sem['actions'],
        'dp':sem['layers'],
        'decisions':decisions,
        'validator_errors':errors,
        'semantic_reason_codes':reason_codes,
        'frame_status':frame_status,
        'operating_mode':'SECOND_LAYER_LOADING' if second_layer_signal else 'FIRST_LAYER_LOADING',
        'second_layer_signal':bool(second_layer_signal),
        'total_dp_cost':sum(float(x.get('best_score',0.0) or 0.0) for x in sem['layers'].values()),
        'timing_ms':{
            'geometry_ms':(t_geom-t0)*1000,
            'candidate_ms':(t_cand-t_geom)*1000,
            'anchor_diagnostic_ms':(t_anchor-t_cand)*1000,
            'semantic_solver_ms':(t_sem-t_anchor)*1000,
            'decision_ms':(t_end-t_sem)*1000,
            'total_ms':(t_end-t0)*1000,
        },
    }


def _next_gid_for_new(d: Decision, tracks):
    if d.layer==1 and d.slot is not None:
        preferred=int(d.slot)
    elif d.layer==2 and d.slot is not None:
        preferred=10+int(d.slot)
    else:
        preferred=max([int(t.global_id) for t in tracks if isinstance(t.global_id,(int,np.integer))]+[-1])+1
    used={int(t.global_id) for t in tracks if isinstance(t.global_id,(int,np.integer))}
    if preferred not in used:
        return preferred
    g=max(used|{-1})+1
    while g in used: g+=1
    return g


def _visibility_from_observation(old: TrackState, o: Observation, cfg: dict):
    """Infer visibility only when current points exist.

    A history track with no current instance is NEVER called OCCLUDED.  The user
    explicitly defined occlusion as a partial-but-nonzero observation.  Point-count
    ratio is only a lightweight proxy in V0.4 and remains conservative.
    """
    vc=cfg.get('visibility',{})
    ref=max(int(old.reference_point_count or 0), int(o.point_count), 1) if old.reference_point_count<=0 else max(int(old.reference_point_count),1)
    ratio=float(o.point_count)/ref
    # A top-camera-only acquisition is a sensor-view limitation, not object occlusion.
    # Keep these semantics separate even when point count is much lower than a prior
    # multi-view frame. OCCLUDED remains reserved for positive partial evidence that
    # is not explained by the known/detected top-only view shape.
    if getattr(o,'observation_mode','MULTI_VIEW')=='TOP_DOWN_Z':
        return vc.get('view_partial_state','VIEW_PARTIAL'), ratio
    if ratio <= float(vc.get('occluded_point_ratio_max',0.52)):
        return 'OCCLUDED', ratio
    if ratio <= float(vc.get('partial_point_ratio_max',0.78)):
        return 'PARTIAL_VISIBLE', ratio
    return 'VISIBLE', ratio


def commit_step(previous_tracks, result, cfg):
    """Commit matcher output to persistent TrackState.

    OCCLUDED now requires a matched, non-empty current instance.  A skipped history
    track has zero current points and therefore becomes UNOBSERVED, never OCCLUDED.
    """
    pmap={t.global_id:t for t in previous_tracks}
    obs=result['observations']
    new_tracks=[]; committed=[]; used_gids=set()
    alpha=float(cfg.get('track_update',{}).get('stable_geometry_alpha',0.05))
    qmin=float(cfg.get('track_update',{}).get('stable_geometry_quality_min',0.80))

    for d in result['decisions']:
        if d.state=='MATCHED':
            old=pmap[d.global_id]; o=obs[d.instance_id]
            vis,ratio=_visibility_from_observation(old,o,cfg)
            a=alpha if o.quality>=qmin and vis=='VISIBLE' and getattr(o,'observation_mode','MULTI_VIEW')!='TOP_DOWN_Z' else 0.0
            ref=max(int(old.reference_point_count or 0),int(o.point_count)) if vis=='VISIBLE' else int(old.reference_point_count or o.point_count)
            foundation_lock=(result.get('operating_mode')=='SECOND_LAYER_LOADING' and old.layer==1 and bool(cfg.get('double_layer',{}).get('freeze_layer1_trackstate_when_active',True)))
            tr=TrackState(
                global_id=old.global_id, layer=old.layer, slot=old.slot, cluster=old.cluster,
                stable_diameter=(float(old.stable_diameter) if foundation_lock else float((1-a)*old.stable_diameter+a*o.diameter)),
                stable_length=(float(old.stable_length) if foundation_lock else float((1-a)*old.stable_length+a*o.length)),
                center=(np.array(old.center,dtype=float) if foundation_lock else np.array(o.center,dtype=float)),
                yaw_deg=(float(old.yaw_deg) if foundation_lock else float(o.yaw_deg)),
                visibility=vis, last_instance_id=o.instance_id,
                confidence=float(d.confidence), reference_point_count=ref,
            )
            new_tracks.append(tr); used_gids.add(tr.global_id); committed.append(d)
        elif d.state=='NEW':
            gid=_next_gid_for_new(d,new_tracks+previous_tracks)
            o=obs[d.instance_id]
            d=Decision(d.instance_id,gid,'NEW',d.confidence,d.layer,d.slot,d.cluster,d.reason,d.match_cost,d.hint_id)
            tr=TrackState(gid,d.layer,d.slot,d.cluster,float(o.diameter),float(o.length),np.array(o.center,dtype=float),float(o.yaw_deg),'VISIBLE',o.instance_id,d.confidence,int(o.point_count))
            new_tracks.append(tr); used_gids.add(gid); committed.append(d)
        elif d.global_id is not None and d.global_id in pmap:
            old=pmap[d.global_id]
            tr=_clone_track(old)
            if d.state=='UNCERTAIN' and d.instance_id is not None:
                # The current instance is real evidence, but identity is not safe enough to
                # commit. Keep the last reliable TrackState untouched while exposing the
                # candidate current observation as UNCERTAIN to the UI/debugger.
                tr.visibility='UNCERTAIN'; tr.last_instance_id=None; tr.confidence=0.0
                new_tracks.append(tr); used_gids.add(tr.global_id); committed.append(d)
            else:
                # Zero current points are not occlusion in the production model.
                tr.visibility='UNOBSERVED'; tr.last_instance_id=None; tr.confidence=float(d.confidence)
                new_tracks.append(tr); used_gids.add(tr.global_id)
                committed.append(Decision(None,d.global_id,'UNOBSERVED',d.confidence,d.layer,d.slot,d.cluster,'zero current points; abnormal missing observation, never OCCLUDED',d.match_cost,d.hint_id))
        else:
            committed.append(d)

    for old in previous_tracks:
        if old.global_id not in used_gids:
            tr=_clone_track(old); tr.visibility='UNOBSERVED'; tr.last_instance_id=None; tr.confidence=0.0
            new_tracks.append(tr)
            committed.append(Decision(None,old.global_id,'UNOBSERVED',0.0,old.layer,old.slot,old.cluster,'old track preserved; zero current points'))

    new_tracks.sort(key=lambda t:(t.layer,t.slot))
    return new_tracks, committed

def movement_rows(previous_tracks, current_tracks, committed_decisions, observations):
    prev={t.global_id:t for t in previous_tracks}
    cmap={t.global_id:t for t in current_tracks}
    dmap={d.global_id:d for d in committed_decisions if d.global_id is not None}
    rows=[]
    for gid,t in sorted(cmap.items(), key=lambda kv:(kv[1].layer,kv[1].slot)):
        d=dmap.get(gid); relation=d.state if d else ('INITIAL' if not previous_tracks else 'MATCHED')
        p=prev.get(gid)
        # An UNCERTAIN decision may point to a real current instance. Show that evidence
        # to the operator, but do not let it update the persistent TrackState.
        uncertain_obs = observations.get(d.instance_id) if d is not None and d.state=='UNCERTAIN' and d.instance_id is not None else None
        foundation_obs = observations.get(d.instance_id) if d is not None and d.state=='MATCHED' and d.instance_id is not None and 'FIRST_LAYER_FOUNDATION_LOCK' in (d.reason or '') else None
        if uncertain_obs is not None:
            c=np.asarray(uncertain_obs.center,dtype=float); yaw=float(uncertain_obs.yaw_deg); iid=uncertain_obs.instance_id
            delta=np.asarray(c)-np.asarray(p.center) if p is not None else None
            delta_list=[float(x) for x in delta] if delta is not None else None
            disp=float(np.linalg.norm(delta)) if delta is not None else None
            dyaw=abs((yaw-float(p.yaw_deg)+180)%360-180) if p is not None else None
            visibility='UNCERTAIN'; quality=0.0
        elif foundation_obs is not None:
            c=np.asarray(foundation_obs.center,dtype=float); yaw=float(foundation_obs.yaw_deg); iid=foundation_obs.instance_id
            delta=np.asarray(c)-np.asarray(p.center) if p is not None else None
            delta_list=[float(x) for x in delta] if delta is not None else None
            disp=float(np.linalg.norm(delta)) if delta is not None else None
            dyaw=abs((yaw-float(p.yaw_deg)+180)%360-180) if p is not None else None
            visibility=t.visibility; quality=float(t.confidence)
        else:
            c=np.asarray(t.center,dtype=float); yaw=float(t.yaw_deg)
            if p is not None and relation=='MATCHED':
                delta=np.asarray(c)-np.asarray(p.center); disp=float(np.linalg.norm(delta))
                dyaw=abs((yaw-float(p.yaw_deg)+180)%360-180); delta_list=[float(x) for x in delta]
            else:
                delta_list=None; disp=None; dyaw=None
            iid=t.last_instance_id if t.visibility not in ('UNOBSERVED','UNCERTAIN') else None
            visibility=t.visibility; quality=float(t.confidence)
        rows.append({
            'global_id':int(gid) if isinstance(gid,(int,np.integer)) else gid,
            'instance_id':iid,'state':relation,'visibility':visibility,'layer':t.layer,'slot':t.slot,'cluster':t.cluster,
            'center':[float(x) for x in c], 'axis':_axis_from_yaw(yaw),
            'yaw_deg':yaw,'radius':float(t.stable_diameter/2.0),
            'diameter':float(t.stable_diameter),'length':float(t.stable_length),
            'point_count':int(observations[iid].point_count) if iid is not None and iid in observations else 0,
            'reference_point_count':int(t.reference_point_count),
            'visibility_ratio':(float(observations[iid].point_count)/max(int(t.reference_point_count),1)) if iid is not None and iid in observations else 0.0,
            'quality':quality,'delta_xyz':delta_list,'displacement':disp,'delta_yaw_deg':dyaw,
            'observation_mode':(getattr(observations[iid],'observation_mode','MULTI_VIEW') if iid is not None and iid in observations else 'NONE'),
            'topdown_likelihood':(float(getattr(observations[iid],'topdown_likelihood',0.0)) if iid is not None and iid in observations else 0.0),
            'center_method':(getattr(observations[iid],'center_method','') if iid is not None and iid in observations else ''),
            'reason':d.reason if d else '',
            'supported_by':([int(t.slot),int(t.slot)+1] if int(t.layer)==2 else None),
            'foundation_locked':bool(int(t.layer)==1 and d is not None and 'FIRST_LAYER_FOUNDATION_LOCK' in (d.reason or '')),
        })
    for d in committed_decisions:
        if d.global_id is not None or d.instance_id is None: continue
        o=observations[d.instance_id]
        rows.append({
            'global_id':None,'instance_id':o.instance_id,'state':'UNCERTAIN','visibility':'VISIBLE','layer':o.layer_guess,'slot':d.slot,'cluster':d.cluster,
            'center':[float(x) for x in o.center],'axis':_axis_from_yaw(o.yaw_deg),'yaw_deg':float(o.yaw_deg),
            'radius':float(o.diameter/2),'diameter':float(o.diameter),'length':float(o.length),'point_count':int(o.point_count),'quality':float(o.quality),
            'delta_xyz':None,'displacement':None,'delta_yaw_deg':None,
            'observation_mode':getattr(o,'observation_mode','MULTI_VIEW'),'topdown_likelihood':float(getattr(o,'topdown_likelihood',0.0)),'center_method':getattr(o,'center_method',''),
            'reason':d.reason,'supported_by':None,'foundation_locked':False,
        })
    return rows

def neighbor_gaps(track_rows, cfg):
    contact=float(cfg.get('gap',{}).get('contact_threshold_m',0.06))
    gap_thr=float(cfg.get('gap',{}).get('clear_gap_threshold_m',0.15))
    by={}
    for r in track_rows:
        if r['global_id'] is None: continue
        by.setdefault((r['layer'],r['cluster']),[]).append(r)
    out=[]
    for (layer,cluster),rows in by.items():
        if cluster not in ('left','right','full_L1','layer2'): continue
        rows=sorted(rows,key=lambda r:r['slot'])
        for a,b in zip(rows,rows[1:]):
            if b['slot']-a['slot']!=1: continue
            ya=math.radians(a['yaw_deg']); yb=math.radians(b['yaw_deg'])
            av=np.array([math.sin(ya),math.cos(ya)])+np.array([math.sin(yb),math.cos(yb)])
            if np.linalg.norm(av)<1e-6: av=np.array([0.0,1.0])
            av=av/np.linalg.norm(av); n=np.array([av[1],-av[0]])
            sep=abs(float(np.dot(np.asarray(b['center'][:2])-np.asarray(a['center'][:2]),n)))
            gap=float(sep-(a['diameter']+b['diameter'])/2.0)
            if a.get('visibility') in ('OCCLUDED','PARTIAL_VISIBLE','UNOBSERVED','UNCERTAIN') or b.get('visibility') in ('OCCLUDED','PARTIAL_VISIBLE','UNOBSERVED','UNCERTAIN') or a['state']=='UNCERTAIN' or b['state']=='UNCERTAIN':
                state='UNCERTAIN'
            elif gap<=contact: state='CONTACT'
            elif gap>=gap_thr: state='GAP'
            else: state='UNCERTAIN'
            out.append({'left_global_id':a['global_id'],'right_global_id':b['global_id'],'layer':layer,'cluster':cluster,'gap_m':gap,'state':state})
    return out


@dataclass
class TrackingSession:
    session_id: str
    sequence_id: str
    relation_mode: str
    frames: list[str]
    current_index: int
    tracks: list[TrackState]
    first_tracks: list[TrackState]
    current_file: str
    previous_file: Optional[str]
    last_rows: list[dict]
    first_rows: list[dict] = field(default_factory=list)
    previous_rows: list[dict] = field(default_factory=list)
    last_debug: dict = field(default_factory=dict)
    id_map: dict = field(default_factory=dict)  # predicted global_id -> benchmark GT identity, evaluation only
    editor_models: dict = field(default_factory=dict)
    editor_gt: dict = field(default_factory=dict)
    editor_actions: dict = field(default_factory=dict)
    editor_hints: dict = field(default_factory=dict)
    last_tracker_state_before: list[dict] = field(default_factory=list)
    last_algorithm_result: dict = field(default_factory=dict)
    second_layer_active: bool = False
    stats: dict = field(default_factory=lambda:{'frames_processed':1,'new':0,'occluded':0,'unobserved':0,'uncertain':0,'wrong_id':0})


class SessionManager:
    def __init__(self, root: Path, cfg: dict):
        self.root=Path(root); self.cfg=cfg; self.ds=SyntheticDatasetAdapter(root)
        self.sessions={}; self.editor=InteractiveSceneEditor(self.root)
        self.frame_manifest=self._read(self.root/'labels/frame_manifest.csv')
        self.sequence_index=self._read(self.root/'labels/sequence_index.csv')

    @staticmethod
    def _read(p):
        with open(p,'r',encoding='utf-8-sig',newline='') as f: return list(csv.DictReader(f))

    def sequences(self):
        out=[]
        for s in self.sequence_index:
            frames=sorted([r for r in self.frame_manifest if r['sequence_id']==s['sequence_id']],key=lambda r:int(r['frame_index']))
            out.append({
                'sequence_id':s['sequence_id'],'relation_mode':s['relation_mode'],'purpose':s['purpose'],
                'num_frames':len(frames),'first_file':frames[0]['file'] if frames else s['first_file'],
                'track_mode':'continuous' if s['relation_mode']=='temporal_adjacent' else ('baseline_compare' if s['relation_mode']=='paired_to_baseline' else 'single_or_independent'),
            })
        return out

    def _frames(self, sequence_id):
        return [r['file'] for r in sorted([x for x in self.frame_manifest if x['sequence_id']==sequence_id],key=lambda r:int(r['frame_index']))]

    def _evaluate_rows(self,file,rows,id_map,learn_initial=False,learn_new=False):
        """Evaluate physical identity while allowing tracker ID numbers to be arbitrary.

        On the first frame we bind tracker IDs to GT physical identities by the
        observed instance.  When a correctly detected NEW object is born, its new
        tracker ID is bound to that GT identity.  Later frames check that the same
        tracker ID keeps pointing to the same physical object.  Thus ID10 vs GT20
        is not treated as an error merely because the numeric labels differ.
        """
        gt={int(float(r['instance_id'])):int(r['global_id']) for r in self.ds.frame_rows(file) if r['instance_id']!=''}
        wrong=[]; correct=0; assigned=0
        for r in rows:
            iid=r['instance_id']; gid=r['global_id']
            if iid is None or gid is None or iid not in gt: continue
            gid=int(gid); gt_gid=gt[iid]; assigned+=1
            if learn_initial and gid not in id_map:
                id_map[gid]=gt_gid
            elif learn_new and r.get('state')=='NEW' and gid not in id_map:
                # Birth is a new identity; bind the tracker-created number to GT.
                if gt_gid not in id_map.values(): id_map[gid]=gt_gid
            mapped=id_map.get(gid)
            if mapped==gt_gid: correct+=1
            elif mapped is not None: wrong.append({'instance_id':iid,'pred_global_id':gid,'mapped_gt_global_id':mapped,'gt_global_id':gt_gid})
        gt_occ=[int(r['global_id']) for r in self.ds.frame_rows(file) if r['visibility']=='OCCLUDED']
        return {'assigned':assigned,'correct':correct,'wrong_id_count':len(wrong),'wrong_ids':wrong,'gt_instance_map':gt,'gt_occluded_ids':gt_occ,'id_map':dict(id_map)}

    def _update_editor_models_from_dataset(self,sess,file,rows):
        """Editor-only physical prototype bookkeeping; never used by matcher."""
        gt_by_iid={int(float(r['instance_id'])):r for r in self.ds.frame_rows(file) if r.get('instance_id','')!=''}
        for pr in rows:
            if pr.get('global_id') is None or pr.get('instance_id') is None: continue
            gr=gt_by_iid.get(int(pr['instance_id']))
            if not gr: continue
            key=self.editor.source_to_key.get(gr.get('source_prototype',''),'prototype_00')
            sess.editor_models[int(pr['global_id'])]={'prototype':key,'diameter':float(pr['diameter']),'length':float(pr['length'])}

    def start(self,sequence_id):
        frames=self._frames(sequence_id)
        if not frames: raise KeyError(sequence_id)
        seq=next(x for x in self.sequence_index if x['sequence_id']==sequence_id)
        inst=self.ds.load_instances(frames[0]); obs=observe_frame(inst,self.cfg)
        second_layer_signal=bool(self.ds.second_layer_signal(frames[0]))
        tracks,decisions=initialize_tracks(obs,self.cfg,second_layer_signal=second_layer_signal)
        rows=movement_rows([],tracks,decisions,obs)
        id_map={}
        ev=self._evaluate_rows(frames[0],rows,id_map,learn_initial=True)
        sid=uuid.uuid4().hex[:12]
        sess=TrackingSession(sid,sequence_id,seq['relation_mode'],frames,0,tracks,clone_tracks(tracks),frames[0],None,rows,list(rows),[],{'stage':'INITIAL','observations':[o.json() for o in obs.values()],'second_layer_signal':second_layer_signal},id_map)
        sess.second_layer_active=second_layer_signal
        sess.editor_models=self.editor.bootstrap_models(self.ds.frame_rows(frames[0]),id_map)
        sess.stats['wrong_id']+=ev['wrong_id_count']
        self.sessions[sid]=sess
        return sess,ev

    def next(self,sid):
        sess=self.sessions[sid]
        if sess.current_index>=len(sess.frames)-1:
            return sess,None,True
        ni=sess.current_index+1; cur=sess.frames[ni]
        old_rows=list(sess.last_rows)
        # Temporal sequences use the algorithm's last committed state. Baseline-variant
        # datasets intentionally compare every variant against F00. Independent sets
        # are reinitialized per frame because they do not define physical continuity.
        if sess.relation_mode=='temporal_adjacent':
            history=clone_tracks(sess.tracks); prev_file=sess.current_file
            sess.last_tracker_state_before=[t.json() for t in history]
            inst=self.ds.load_instances(cur); hints=self.ds.robot_hints(cur)
            second_layer_signal=bool(sess.second_layer_active or self.ds.second_layer_signal(cur))
            result=match_from_tracks(history,inst,hints,self.cfg,second_layer_signal=second_layer_signal)
            sess.second_layer_active=second_layer_signal
            new_tracks,committed=commit_step(history,result,self.cfg)
            rows=movement_rows(history,new_tracks,committed,result['observations'])
            sess.tracks=new_tracks
        elif sess.relation_mode=='paired_to_baseline':
            history=clone_tracks(sess.first_tracks); prev_file=sess.frames[0]
            sess.last_tracker_state_before=[t.json() for t in history]
            inst=self.ds.load_instances(cur); hints=self.ds.robot_hints(cur)
            second_layer_signal=bool(sess.second_layer_active or self.ds.second_layer_signal(cur))
            result=match_from_tracks(history,inst,hints,self.cfg,second_layer_signal=second_layer_signal)
            sess.second_layer_active=second_layer_signal
            new_tracks,committed=commit_step(history,result,self.cfg)
            rows=movement_rows(history,new_tracks,committed,result['observations'])
            # Do not change baseline reference; current display still uses new_tracks.
            sess.tracks=new_tracks
        else:
            prev_file=None
            inst=self.ds.load_instances(cur); obs=observe_frame(inst,self.cfg)
            second_layer_signal=bool(self.ds.second_layer_signal(cur))
            new_tracks,committed=initialize_tracks(obs,self.cfg,second_layer_signal=second_layer_signal)
            sess.second_layer_active=second_layer_signal
            result={'observations':obs,'candidates':[],'anchors':[],'anchor_audit':[],'actions':[],'dp':{},'validator_errors':[],'timing_ms':{},'total_dp_cost':0}
            rows=movement_rows([],new_tracks,committed,obs); sess.tracks=new_tracks

        sess.previous_file=prev_file; sess.current_file=cur; sess.current_index=ni; sess.last_rows=rows
        sess.previous_rows=list(sess.first_rows if sess.relation_mode=='paired_to_baseline' else old_rows) if prev_file else []
        sess.last_debug={
            'stage':'MATCH','candidates':[c.json() for c in result['candidates']],
            'anchors':[{'global_id':g,'instance_id':i,'cost':c} for g,i,c in result['anchors']],
            'anchor_audit':result['anchor_audit'],'actions':result['actions'],'dp':result['dp'],
            'topdown_layer_assignment':result.get('topdown_layer_assignment',{}),
            'validator_errors':result['validator_errors'],'timing_ms':result['timing_ms'],
            'frame_status':result.get('frame_status','OK'),'semantic_reason_codes':result.get('semantic_reason_codes',[]),
        }
        sess.last_algorithm_result={**sess.last_debug,'rows':rows}
        ev=self._evaluate_rows(cur,rows,sess.id_map,learn_new=True)
        self._update_editor_models_from_dataset(sess,cur,rows)
        sess.stats['frames_processed']+=1
        sess.stats['wrong_id']+=ev['wrong_id_count']
        sess.stats['new']+=sum(r['state']=='NEW' for r in rows)
        sess.stats['occluded']+=sum(r.get('visibility')=='OCCLUDED' for r in rows)
        sess.stats['unobserved']=sess.stats.get('unobserved',0)+sum(r.get('visibility')=='UNOBSERVED' for r in rows)
        sess.stats['uncertain']+=sum(r['state']=='UNCERTAIN' for r in rows)
        return sess,ev,False

    def get(self,sid): return self.sessions[sid]
    def reset(self,sid): self.sessions.pop(sid,None)


    def editor_seed(self, sid):
        sess=self.sessions[sid]
        xs=[float(r['center'][0]) for r in sess.last_rows if r.get('global_id') is not None]
        # The synthetic truck convention uses X≈0..15 m and coil centers near Y=0.
        # Keep a stable viewport so dragging between sequences feels consistent.
        x_min=min(0.0,(min(xs)-1.5) if xs else 0.0); x_max=max(15.0,(max(xs)+1.5) if xs else 15.0)
        return {
            'current_file':sess.current_file,
            'objects':sess.last_rows,
            'prototypes':self.editor.prototypes(),
            'viewport':{'x_min':x_min,'x_max':x_max,'y_min':-2.0,'y_max':2.0},
            'second_layer_active':bool(sess.second_layer_active),
            'rules':{
                'old_removal_supported':False,
                'full_occlusion_supported':False,
                'occlusion_requires_partial_points':True,
                'layer1_max_slots':int(self.cfg['business']['layer1_max_slots']),
                'z_locked_by_default':True,
                'position_edit_mode':'drag_xy',
                'motion_roi':{
                    'x_default_m':float(self.cfg.get('motion_roi',{}).get('x_default_m',1.5)),
                    'x_normal_max_m':float(self.cfg.get('motion_roi',{}).get('x_absolute_max_m',2.0)),
                    'x_hard_max_m':float(self.cfg.get('motion_roi',{}).get('x_hard_max_m',self.cfg.get('motion_roi',{}).get('x_absolute_max_m',2.0))),
                    'y_max_m':float(self.cfg.get('motion_roi',{}).get('y_absolute_max_m',self.cfg.get('motion_roi',{}).get('y_default_m',0.4))),
                    'z_max_m':float(self.cfg.get('motion_roi',{}).get('z_absolute_max_m',self.cfg.get('motion_roi',{}).get('z_default_m',0.3))),
                },
                'view_modes':['MULTI_VIEW','TOP_DOWN_Z'],
                'double_layer':{
                    'enabled':bool(self.cfg.get('double_layer',{}).get('enabled',False)),
                    'second_layer_active':bool(sess.second_layer_active),
                    'requires_full_layer1':True,
                    'layer1_required_slots':int(self.cfg.get('double_layer',{}).get('layer1_required_slots',10)),
                    'support_slot_definition':'upper slot k is supported by L1[k] + L1[k+1]',
                    'foundation_hard_xy_m':float(self.cfg.get('double_layer',{}).get('layer1_foundation_hard_xy_m',0.12)),
                },
                'topdown_rule':'camera looks along world -Z; global XY z-buffer retains top-facing surface only',
            }
        }

    def create_next(self, sid, request):
        sess=self.sessions[sid]
        history=clone_tracks(sess.tracks)
        sess.last_tracker_state_before=[t.json() for t in history]
        created=self.editor.create(sess,request,self.cfg)
        frame_no=sess.current_index+1
        # Keep every PLY flat in the dataset root for visual inspection.
        rel=Path(f'USER_{sid}_F{frame_no:03d}.ply')
        abs_path=self.root/rel
        from .editor import write_scene_ply
        npoints=write_scene_ply(abs_path,created['scene'])
        inst=self.ds.load_instances(str(rel))
        requested_signal=request.get('second_layer_signal',None)
        implied_signal=any(int(g.get('layer',1))==2 for g in created.get('gt',[])) or any(int(t.layer)==2 for t in history)
        second_layer_signal=bool(sess.second_layer_active or implied_signal or requested_signal is True)
        result=match_from_tracks(history,inst,created['hints'],self.cfg,second_layer_signal=second_layer_signal)
        sess.second_layer_active=second_layer_signal
        new_tracks,committed=commit_step(history,result,self.cfg)
        rows=movement_rows(history,new_tracks,committed,result['observations'])
        prev_file=sess.current_file; old_rows=list(sess.last_rows)
        sess.frames.append(str(rel)); sess.previous_file=prev_file; sess.current_file=str(rel); sess.current_index=len(sess.frames)-1
        sess.previous_rows=old_rows; sess.tracks=new_tracks; sess.last_rows=rows
        sess.editor_gt[str(rel)]=created['gt']; sess.editor_actions[str(rel)]=created['actions']; sess.editor_hints[str(rel)]=[h.json() for h in created['hints']]
        sess.last_debug={'stage':'INTERACTIVE_MATCH','candidates':[c.json() for c in result['candidates']],
            'anchors':[{'global_id':g,'instance_id':i,'cost':c} for g,i,c in result['anchors']],
            'anchor_audit':result['anchor_audit'],'actions':result['actions'],'dp':result['dp'],
            'topdown_layer_assignment':result.get('topdown_layer_assignment',{}),
            'validator_errors':result['validator_errors'],'timing_ms':result['timing_ms'],
            'frame_status':result.get('frame_status','OK'),'semantic_reason_codes':result.get('semantic_reason_codes',[]),
            'editor_warnings':created['warnings'],'editor_seed':created['seed'],'generated_points':npoints}
        sess.last_algorithm_result={**sess.last_debug,'rows':rows}
        # User-authored GT identities are the current tracker-visible IDs by definition. Evaluate directly.
        gt_by_iid={int(r['instance_id']):int(r['global_id']) for r in created['gt']}
        wrong=[]
        for r in rows:
            if r['instance_id'] is None or r['global_id'] is None: continue
            gt_gid=gt_by_iid.get(int(r['instance_id']))
            if gt_gid is not None and int(r['global_id'])!=gt_gid:
                wrong.append({'instance_id':r['instance_id'],'pred_global_id':r['global_id'],'gt_global_id':gt_gid})
        ev={'wrong_id_count':len(wrong),'wrong_ids':wrong,'assigned':sum(r['instance_id'] is not None and r['global_id'] is not None for r in rows)}
        sess.stats['frames_processed']+=1; sess.stats['wrong_id']+=len(wrong)
        sess.stats['new']+=sum(r['state']=='NEW' for r in rows)
        sess.stats['occluded']+=sum(r.get('visibility')=='OCCLUDED' for r in rows)
        sess.stats['unobserved']=sess.stats.get('unobserved',0)+sum(r.get('visibility')=='UNOBSERVED' for r in rows)
        sess.stats['uncertain']+=sum(r['state']=='UNCERTAIN' for r in rows)
        return sess,created,ev

    def current_gt_rows(self, sess):
        if sess.current_file in sess.editor_gt:
            rows=[]
            for r in sess.editor_gt[sess.current_file]:
                rows.append({**r,'file':sess.current_file})
            return rows
        return self.ds.frame_rows(sess.current_file)

    def reset(self,sid):
        self.sessions.pop(sid,None)

    def get(self,sid):
        return self.sessions[sid]
