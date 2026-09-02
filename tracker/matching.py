from __future__ import annotations
import math
from collections import defaultdict
import numpy as np
from .models import Candidate, Decision, TrackState, Observation, NewCoilHint

def _angle_diff(a,b): return abs((a-b+180)%360-180)


def _double_layer_active(cfg:dict)->bool:
    return bool(cfg.get('double_layer',{}).get('enabled',False) and cfg.get('_runtime',{}).get('second_layer_signal',False))


def _layer1_support_centers(tracks:list[TrackState]):
    """Return semantic support valleys from the locked full first layer.

    Upper semantic slot k is defined by the adjacent first-layer pair (k,k+1).
    The XY support reference is the midpoint of the two persistent lower centers.
    """
    by={int(t.slot):t for t in tracks if int(t.layer)==1}
    out={}
    for k in range(9):
        if k in by and k+1 in by:
            c=(np.asarray(by[k].center,dtype=float)+np.asarray(by[k+1].center,dtype=float))/2.0
            out[k]=c
    return out


def _nearest_support_slot(tracks:list[TrackState], center):
    supports=_layer1_support_centers(tracks)
    if not supports: return None,float('inf'),None
    c=np.asarray(center,dtype=float)
    best=min(supports.items(),key=lambda kv:float(np.linalg.norm(c[:2]-np.asarray(kv[1])[:2])))
    slot=int(best[0]); ref=np.asarray(best[1],dtype=float)
    return slot,float(np.linalg.norm(c[:2]-ref[:2])),ref

def build_candidates(tracks:list[TrackState], observations:dict[int,Observation], cfg:dict):
    c=cfg['matching']; roi=cfg['motion_roi']; out=[]; accepted={}
    rx=float(roi['x_default_m']); ry=float(roi['y_default_m']); rz=float(roi['z_default_m'])
    for t in tracks:
        for iid,o in observations.items():
            reasons=[]
            dx,dy,dz=o.center-t.center
            if o.layer_guess!=t.layer: reasons.append('LAYER_MISMATCH')
            if abs(dx)>rx: reasons.append('OUTSIDE_X_ROI')
            if abs(dy)>ry: reasons.append('OUTSIDE_Y_ROI')
            if abs(dz)>rz: reasons.append('OUTSIDE_Z_ROI')
            dD=abs(o.diameter-t.stable_diameter); dL=abs(o.length-t.stable_length); dyaw=_angle_diff(o.yaw_deg,t.yaw_deg)
            if dD>float(c['diameter_gate_m']): reasons.append('DIAMETER_GATE')
            if dL>float(c['length_gate_m']): reasons.append('LENGTH_GATE')
            # yaw intentionally remains soft in V1.
            ok=not reasons
            comps={
                'position':float((dx/rx)**2+(dy/ry)**2+(dz/rz)**2),
                'diameter':float((dD/float(c['diameter_scale_m']))**2),
                'length':float((dL/float(c['length_scale_m']))**2),
                'yaw':float((dyaw/float(c['yaw_scale_deg']))**2),
                'quality':float(1-o.quality),
            }
            cost=(float(c['w_position'])*comps['position']+float(c['w_diameter'])*comps['diameter']+
                  float(c['w_length'])*comps['length']+float(c['w_yaw'])*comps['yaw']+
                  float(c['w_quality'])*comps['quality']) if ok else None
            cand=Candidate(t.global_id,iid,ok,None if cost is None else float(cost),reasons,comps)
            out.append(cand)
            if ok: accepted[(t.global_id,iid)]=cand
    return out,accepted



def select_semantic_rank_anchors(tracks, observations, accepted, hints, cfg):
    """Hard semantic anchors for a fully observed, no-birth layer.

    If every committed history track in a layer was visible, current observation
    count is unchanged, and the robot reports no birth for this step, then the
    business model says there is no removal and same-layer order cannot reverse.
    In that case rank-by-X is identity, even when an entire cluster translates
    by more than half a coil spacing and nearest position becomes misleading.

    We still require every rank pair to pass the normal physical gates and the
    rank displacements to be collectively coherent.  If these checks fail we do
    not force anchors; the conservative DP/UNCERTAIN path remains available.
    """
    mc=cfg.get('matching',{})
    if hints:
        return [], []
    max_std=float(mc.get('rank_lock_dx_std_max_m',0.40))
    max_span=float(mc.get('rank_lock_dx_span_max_m',0.85))
    anchors=[]; audit=[]
    layers=sorted(set([t.layer for t in tracks]+[o.layer_guess for o in observations.values()]))
    for layer in layers:
        H=sorted([t for t in tracks if t.layer==layer], key=lambda t:t.slot)
        O=sorted([o for o in observations.values() if o.layer_guess==layer], key=lambda o:o.center[0])
        rec={'layer':layer,'history_count':len(H),'observation_count':len(O),'qualified':False,'reason':''}
        if not H or len(H)!=len(O):
            rec['reason']='COUNT_CHANGED'; audit.append(rec); continue
        if any(t.visibility in ('UNOBSERVED','UNCERTAIN') or t.last_instance_id is None for t in H):
            rec['reason']='HISTORY_HAS_NO_CURRENT_POINTS'; audit.append(rec); continue
        pairs=[]; dxs=[]; bad=[]
        for h,o in zip(H,O):
            c=accepted.get((h.global_id,o.instance_id))
            if c is None:
                bad.append((h.global_id,o.instance_id)); continue
            pairs.append((h.global_id,o.instance_id,c.cost)); dxs.append(float(o.center[0]-h.center[0]))
        if bad:
            rec['reason']='RANK_PAIR_FAILED_GATE'; rec['failed_pairs']=bad; audit.append(rec); continue
        if dxs:
            std=float(np.std(dxs)); span=float(max(dxs)-min(dxs))
        else:
            std=span=0.0
        rec['dx_mean_m']=float(np.mean(dxs)) if dxs else 0.0; rec['dx_std_m']=std; rec['dx_span_m']=span
        if std>max_std or span>max_span:
            rec['reason']='COLLECTIVE_TRANSLATION_NOT_COHERENT'; audit.append(rec); continue
        rec['qualified']=True; rec['reason']='FULL_VISIBLE_NO_BIRTH_ORDER_LOCK'; audit.append(rec)
        anchors.extend(pairs)
    return anchors,audit

def select_anchors(tracks,observations,accepted,cfg):
    mc=cfg['matching']; anchors=[]; used_tracks=set(); used_obs=set(); audit=[]
    by_obs=defaultdict(list)
    for (gid,iid),cand in accepted.items(): by_obs[iid].append(cand)
    for iid,o in sorted(observations.items(),key=lambda kv:kv[1].center[0]):
        cs=sorted(by_obs.get(iid,[]),key=lambda x:x.cost)
        if not cs: continue
        best=cs[0]; second=cs[1].cost if len(cs)>1 else float('inf'); margin=second-best.cost
        qualifies=(o.quality>=float(mc['anchor_quality_min']) and best.cost<=float(mc['anchor_best_cost_max']) and margin>=float(mc['anchor_margin_min']))
        audit.append({'instance_id':iid,'best_global_id':best.global_id,'best_cost':best.cost,'second_cost':None if math.isinf(second) else second,'margin':None if math.isinf(margin) else margin,'quality':o.quality,'qualifies':qualifies})
        if qualifies and best.global_id not in used_tracks and iid not in used_obs:
            anchors.append((best.global_id,iid,best.cost)); used_tracks.add(best.global_id); used_obs.add(iid)
    # Guard against anchors that would reverse order within a layer: keep monotone anchors only.
    tmap={t.global_id:t for t in tracks}; valid=[]
    for layer in sorted(set(t.layer for t in tracks)):
        aa=[a for a in anchors if tmap[a[0]].layer==layer]
        aa=sorted(aa,key=lambda a:tmap[a[0]].slot)
        last_x=-float('inf')
        for a in aa:
            x=observations[a[1]].center[0]
            if x>last_x: valid.append(a); last_x=x
    return valid,audit

def ordered_dp(tracks,obs_list,accepted,cfg,anchor_pairs=None):
    """Order-preserving alignment. anchor_pairs are treated as zero-ambiguity matches by making incompatible pairs unavailable."""
    mc=cfg['matching']; skip=float(mc['skip_history_cost']); ins=float(mc['insert_current_cost'])
    H=sorted(tracks,key=lambda t:t.slot); O=sorted(obs_list,key=lambda o:o.center[0]); m,n=len(H),len(O)
    anchor_by_gid={g:i for g,i,*_ in (anchor_pairs or [])}; anchor_by_iid={i:g for g,i,*_ in (anchor_pairs or [])}
    inf=1e18; dp=np.full((m+1,n+1),inf); prev=[[None]*(n+1) for _ in range(m+1)]; dp[0,0]=0
    for i in range(m+1):
        for j in range(n+1):
            cur=dp[i,j]
            if cur>=inf: continue
            if i<m:
                h=H[i]
                # Anchored history cannot be skipped.
                if h.global_id not in anchor_by_gid and cur+skip<dp[i+1,j]:
                    dp[i+1,j]=cur+skip; prev[i+1][j]=(i,j,'SKIP_HISTORY',h.global_id,None,skip)
            if j<n:
                o=O[j]
                # Anchored observation cannot be inserted.
                if o.instance_id not in anchor_by_iid and cur+ins<dp[i,j+1]:
                    dp[i,j+1]=cur+ins; prev[i][j+1]=(i,j,'INSERT_CURRENT',None,o.instance_id,ins)
            if i<m and j<n:
                h,o=H[i],O[j]
                cand=accepted.get((h.global_id,o.instance_id))
                anchor_ok=True
                if h.global_id in anchor_by_gid and anchor_by_gid[h.global_id]!=o.instance_id: anchor_ok=False
                if o.instance_id in anchor_by_iid and anchor_by_iid[o.instance_id]!=h.global_id: anchor_ok=False
                if cand and anchor_ok and cur+cand.cost<dp[i+1,j+1]:
                    dp[i+1,j+1]=cur+cand.cost; prev[i+1][j+1]=(i,j,'MATCH',h.global_id,o.instance_id,cand.cost)
    actions=[]; i,j=m,n
    if dp[i,j]>=inf: return [],float('inf'),{'shape':[m,n]}
    while i or j:
        p=prev[i][j]
        if p is None: break
        pi,pj,act,gid,iid,cost=p; actions.append({'action':act,'global_id':gid,'instance_id':iid,'cost':float(cost)}); i,j=pi,pj
    actions.reverse()
    return actions,float(dp[m,n]),{'shape':[m,n],'history_order':[t.global_id for t in H],'observation_order':[o.instance_id for o in O]}

def infer_new_slot(obs,tracks,cfg):
    """Infer the semantic birth slot without reading current-frame GT.

    Layer-1 is modeled as one or two edge-grown clusters.  A left-only history can
    only grow at its inner/right boundary; a right-only history can only grow at
    its inner/left boundary.  With both clusters present, choose the nearer inner
    boundary.  This avoids the old midpoint heuristic incorrectly creating a left
    cluster when only the right cluster exists.
    """
    if obs.layer_guess==2:
        # In the real double-layer loading mode, an upper slot is not an append index:
        # it is the support valley between first-layer slots k and k+1.
        if bool(cfg.get('double_layer',{}).get('enabled',False)):
            req=int(cfg.get('double_layer',{}).get('layer1_required_slots',10))
            l1=[t for t in tracks if t.layer==1]
            if len(l1)==req and {int(t.slot) for t in l1}==set(range(req)):
                slot,err,_=_nearest_support_slot(tracks,obs.center)
                occupied={int(t.slot) for t in tracks if t.layer==2}
                hard=float(cfg.get('double_layer',{}).get('support_slot_hard_center_error_m',0.65))
                if slot is None or err>hard or slot in occupied:
                    return None,None
                return int(slot),'layer2'
        slots=[t.slot for t in tracks if t.layer==2]
        return (max(slots)+1 if slots else 0),'layer2'

    cap=int(cfg['business']['layer1_max_slots'])
    l1=[t for t in tracks if t.layer==1]
    occupied={t.slot for t in l1}
    if len(occupied)>=cap:
        return None,None

    left_tracks=[t for t in l1 if t.cluster in ('left','full_L1')]
    right_tracks=[t for t in l1 if t.cluster=='right']
    left=sorted(t.slot for t in left_tracks)
    right=sorted(t.slot for t in right_tracks)

    # A full-L1 history is not allowed to produce an 11th layer-1 object.
    if any(t.cluster=='full_L1' for t in l1):
        return None,None

    if left and not right:
        slot=max(left)+1
        return (slot,'left') if slot<cap else (None,None)
    if right and not left:
        slot=min(right)-1
        return (slot,'right') if slot>=0 else (None,None)
    if not left and not right:
        # Empty/unknown first layer: use truck midpoint only as a bootstrap rule.
        mid=float(cfg.get('initialization',{}).get('truck_mid_x_m',7.5))
        return (0,'left') if obs.center[0]<=mid else (cap-1,'right')

    # Two clusters: births may only extend one of the two inner boundaries.
    left_inner=max(left_tracks,key=lambda t:t.slot)
    right_inner=min(right_tracks,key=lambda t:t.slot)
    dl=abs(float(obs.center[0])-float(left_inner.center[0]))
    dr=abs(float(obs.center[0])-float(right_inner.center[0]))
    if dl<=dr:
        slot=max(left)+1
        if slot in occupied or slot>=min(right): return None,None
        return slot,'left'
    slot=min(right)-1
    if slot in occupied or slot<=max(left): return None,None
    return slot,'right'

def match_hints(inserts:list[Observation],hints:list[NewCoilHint],cfg):
    tol=float(cfg['new_coil']['position_tolerance_m']); available=set(range(len(hints))); out={}
    for o in sorted(inserts,key=lambda x:x.center[0]):
        best=None
        for k in available:
            h=hints[k]; d=float(np.linalg.norm(o.center-h.target_center)); dD=abs(o.diameter-h.diameter); dL=abs(o.length-h.length)
            score=d/tol + .25*dD/float(cfg['new_coil']['diameter_tolerance_m']) + .25*dL/float(cfg['new_coil']['length_tolerance_m'])
            if best is None or score<best[0]: best=(score,k,d)
        if best and best[2]<=tol:
            out[o.instance_id]=hints[best[1]]; available.remove(best[1])
    return out

def decisions_from_actions(actions,tracks,observations,hints,cfg,accepted):
    tmap={t.global_id:t for t in tracks}; omap=observations
    inserts=[omap[a['instance_id']] for a in actions if a['action']=='INSERT_CURRENT']
    hint_match=match_hints(inserts,hints,cfg)
    decisions=[]; matched_gids=set(); matched_iids=set()
    # First matched observations.
    for a in actions:
        if a['action']!='MATCH': continue
        t=tmap[a['global_id']]; o=omap[a['instance_id']]; matched_gids.add(t.global_id); matched_iids.add(o.instance_id)
        conf=float(np.clip(1.0-a['cost']/max(float(cfg['matching']['skip_history_cost']),1e-6),0.05,0.99))
        decisions.append(Decision(o.instance_id,t.global_id,'MATCHED',conf,t.layer,t.slot,t.cluster,'ordered DP match',a['cost']))
    # Insertions: only commit NEW when robot hint agrees and slot growth is legal.
    for a in actions:
        if a['action']!='INSERT_CURRENT': continue
        o=omap[a['instance_id']]; h=hint_match.get(o.instance_id); slot,cluster=infer_new_slot(o,tracks,cfg)
        if h is not None and slot is not None:
            decisions.append(Decision(o.instance_id,None,'NEW',.95,o.layer_guess,slot,cluster,'unexplained current instance + robot hint + legal slot growth',None,h.hint_id))
        elif h is not None and o.layer_guess==2:
            slot,cluster=infer_new_slot(o,tracks,cfg)
            decisions.append(Decision(o.instance_id,None,'NEW',.92,2,slot,cluster,'unexplained current instance + robot hint; layer2 birth',None,h.hint_id))
        else:
            decisions.append(Decision(o.instance_id,None,'UNCERTAIN',.0,o.layer_guess,slot,cluster,'inserted observation is not safe to confirm as NEW'))
    # Missing history has ZERO current points. By business definition this cannot
    # be called OCCLUDED. Occlusion requires a non-empty partial observation.
    # Semantic bracketing is still recorded as evidence, but the state is UNOBSERVED.
    matched_slots=defaultdict(set)
    for d in decisions:
        if d.state=='MATCHED': matched_slots[d.layer].add(d.slot)
    for a in actions:
        if a['action']!='SKIP_HISTORY': continue
        t=tmap[a['global_id']]
        same=sorted(x.slot for x in tracks if x.layer==t.layer and x.cluster==t.cluster)
        left=any(s<t.slot and s in matched_slots[t.layer] for s in same)
        right=any(s>t.slot and s in matched_slots[t.layer] for s in same)
        reason='zero current points; semantically bracketed by visible neighbors' if left and right else 'zero current points; history preserved conservatively'
        decisions.append(Decision(None,t.global_id,'UNOBSERVED',.55 if left and right else .0,t.layer,t.slot,t.cluster,reason))
    return decisions

# ---------------------------------------------------------------------------
# V0.6 semantic-first constrained solver
# ---------------------------------------------------------------------------

def _hint_layer(h: NewCoilHint, cfg: dict) -> int:
    if h.expected_layer is not None:
        return int(h.expected_layer)
    z_thr=float(cfg.get('geometry',{}).get('layer2_center_z_min_m',1.15))
    return 2 if float(h.target_center[2])>=z_thr else 1


def _candidate_all_map(candidates:list[Candidate]):
    return {(c.global_id,c.instance_id):c for c in candidates}


def _weighted_candidate_cost(cand: Candidate, cfg: dict) -> float:
    mc=cfg['matching']; x=cand.components
    return float(
        float(mc['w_position'])*x['position']+
        float(mc['w_diameter'])*x['diameter']+
        float(mc['w_length'])*x['length']+
        float(mc['w_yaw'])*x['yaw']+
        float(mc['w_quality'])*x['quality']
    )


def _topdown_resolved_center(o:Observation, reference_center, diameter:float, length:float):
    """Recover a plausible cylinder center from a top-only partial surface.

    With known/stable D/L and a semantic identity hypothesis, every visible point
    constrains the unknown cylinder center to an interval along the cylinder normal
    and axis. We choose the point in those feasible intervals closest to the prior
    center (or robot target for NEW). This uses history as a geometric prior without
    pretending the clipped current bbox is a full cylinder.
    """
    c0=np.asarray(o.center,dtype=float).copy()
    if getattr(o,'observation_mode','MULTI_VIEW')!='TOP_DOWN_Z':
        return c0
    yaw=math.radians(float(o.yaw_deg))
    a=np.array([math.sin(yaw),math.cos(yaw)],dtype=float)
    n=np.array([math.cos(yaw),-math.sin(yaw)],dtype=float)
    ref=np.asarray(reference_center,dtype=float)
    R=max(float(diameter)/2.0,1e-6); H=max(float(length)/2.0,1e-6)

    # Visible normal coordinates are a subset of [c_n-R,c_n+R].
    nlo=float(o.normal_max)-R; nhi=float(o.normal_min)+R
    refn=float(ref[:2]@n)
    cn=float(np.clip(refn,nlo,nhi)) if nlo<=nhi else float((o.normal_min+o.normal_max)/2.0)
    # Likewise for the axial extent. Length is usually almost complete from above,
    # but this interval also handles end clipping conservatively.
    slo=float(o.axis_max)-H; shi=float(o.axis_min)+H
    refs=float(ref[:2]@a)
    cs=float(np.clip(refs,slo,shi)) if slo<=shi else float((o.axis_min+o.axis_max)/2.0)
    xy=cn*n+cs*a
    z=float(getattr(o,'top_z',c0[2]+R))-R
    return np.array([xy[0],xy[1],z],dtype=float)


def _pair_cost_from_resolved(t:TrackState,o:Observation,resolved_center,cfg:dict):
    """Pair cost after top-view center recovery.

    In TOP_DOWN_Z, an observed D/L smaller than the stable model is explainable by
    view/upper-layer clipping and must not be treated like a contradictory larger
    object. Over-size remains evidence against the hypothesis.
    """
    mc=cfg['matching'];roi=cfg.get('motion_roi',{})
    dx,dy,dz=np.asarray(resolved_center)-np.asarray(t.center)
    rx=max(float(roi.get('x_default_m',1.5)),1e-6);ry=max(float(roi.get('y_default_m',0.4)),1e-6);rz=max(float(roi.get('z_default_m',0.3)),1e-6)
    top=getattr(o,'observation_mode','MULTI_VIEW')=='TOP_DOWN_Z'
    if top:
        dD=max(0.0,float(o.diameter)-float(t.stable_diameter))
        dL=max(0.0,float(o.length)-float(t.stable_length))
    else:
        dD=abs(float(o.diameter)-float(t.stable_diameter));dL=abs(float(o.length)-float(t.stable_length))
    comps={
      'position':float((dx/rx)**2+(dy/ry)**2+(dz/rz)**2),
      'diameter':float((dD/float(mc['diameter_scale_m']))**2),
      'length':float((dL/float(mc['length_scale_m']))**2),
      'yaw':float((_angle_diff(o.yaw_deg,t.yaw_deg)/float(mc['yaw_scale_deg']))**2),
      'quality':float(1-o.quality),
    }
    return float(float(mc['w_position'])*comps['position']+float(mc['w_diameter'])*comps['diameter']+float(mc['w_length'])*comps['length']+float(mc['w_yaw'])*comps['yaw']+float(mc['w_quality'])*comps['quality']),comps


def _semantic_pair_assessment(t:TrackState,o:Observation,cand:Candidate,cfg:dict):
    """Assess a rank-fixed historical/current pair.

    V0.6 identity is first constrained by layer/order/cardinality. Geometry then
    validates that semantic pairing.  A normal candidate remains MATCHED.  A
    partial-but-positive observation may relax D/L gates because its geometry is
    expected to be biased.  X motion may use the configured absolute safety ROI
    after order/cardinality have made the pairing unique.  Anything beyond the
    safety envelope becomes UNCERTAIN; it never causes later IDs to shift.
    """
    roi=cfg.get('motion_roi',{}); mc=cfg.get('matching',{}); vc=cfg.get('visibility',{})
    resolved_center=_topdown_resolved_center(o,t.center,t.stable_diameter,t.stable_length)
    dx,dy,dz=np.asarray(resolved_center,dtype=float)-np.asarray(t.center,dtype=float)
    x_default=float(roi.get('x_default_m',1.5))
    # x_absolute_max_m is the normal business envelope. V0.7 adds a second hard
    # safety envelope so a small overrun can be reported as abnormal motion without
    # throwing away an otherwise unique identity.
    x_abs=float(roi.get('x_absolute_max_m',x_default))
    x_hard=float(roi.get('x_hard_max_m',x_abs))
    y_abs=float(roi.get('y_absolute_max_m',roi.get('y_default_m',0.40)))
    z_abs=float(roi.get('z_absolute_max_m',roi.get('z_default_m',0.30)))
    ref=max(int(t.reference_point_count or 0),1)
    ratio=float(o.point_count)/ref
    view_partial=(getattr(o,'observation_mode','MULTI_VIEW')=='TOP_DOWN_Z')
    partial_positive=(o.point_count>0 and (ratio<=float(vc.get('partial_point_ratio_max',0.78)) or view_partial))
    reasons=list(cand.reasons)
    hard=[]; relaxed=[]; details=[]
    foundation_locked=False
    if o.layer_guess!=t.layer: hard.append('LAYER_MISMATCH')
    if _double_layer_active(cfg) and int(t.layer)==1:
        foundation_locked=True
        dc=cfg.get('double_layer',{})
        xy=float(np.linalg.norm(np.asarray([dx,dy],dtype=float)))
        zshift=abs(float(dz)); yshift=_angle_diff(float(o.yaw_deg),float(t.yaw_deg))
        is_topdown=(getattr(o,'observation_mode','MULTI_VIEW')=='TOP_DOWN_Z')
        if is_topdown:
            # A locked foundation observed only from +Z may expose a narrow strip whose
            # bbox center/yaw changes substantially as upper coils clip it.  That is an
            # observation-shape effect, not evidence that the physical foundation moved.
            # Use wider translation envelopes and ignore yaw for collapse diagnosis.
            nxy=float(dc.get('layer1_foundation_topdown_normal_xy_m',0.12)); hxy=float(dc.get('layer1_foundation_topdown_hard_xy_m',0.20))
            nz=float(dc.get('layer1_foundation_topdown_normal_z_m',0.15)); hz=float(dc.get('layer1_foundation_topdown_hard_z_m',0.22))
            nyaw=float(dc.get('layer1_foundation_normal_yaw_deg',6.0)); hyaw=float(dc.get('layer1_foundation_hard_yaw_deg',12.0))
            yaw_hard=False; yaw_soft=False
        else:
            nxy=float(dc.get('layer1_foundation_normal_xy_m',0.08)); hxy=float(dc.get('layer1_foundation_hard_xy_m',0.15))
            nz=float(dc.get('layer1_foundation_normal_z_m',0.08)); hz=float(dc.get('layer1_foundation_hard_z_m',0.15))
            nyaw=float(dc.get('layer1_foundation_normal_yaw_deg',6.0)); hyaw=float(dc.get('layer1_foundation_hard_yaw_deg',12.0))
            # Yaw is a secondary diagnostic only when it is very large.  The coil axis
            # estimator can move several degrees with partial surfaces even in multi-view.
            yaw_hard=(yshift>hyaw)
            yaw_soft=(yshift>nyaw)
        if xy>hxy or zshift>hz or yaw_hard:
            hard.append('ABNORMAL_FIRST_LAYER_SHIFT_HARD')
            details.append(f'locked L1 foundation shift xy={xy:.3f}m (hard {hxy:.3f}), dz={zshift:.3f}m (hard {hz:.3f}), dyaw={yshift:.2f}deg (hard {hyaw:.2f}, topdown_yaw_ignored={is_topdown})')
        elif xy>nxy or zshift>nz or yaw_soft:
            relaxed.append('ABNORMAL_FIRST_LAYER_SHIFT')
            details.append(f'locked L1 foundation shift xy={xy:.3f}m (normal {nxy:.3f}), dz={zshift:.3f}m (normal {nz:.3f}), dyaw={yshift:.2f}deg (normal {nyaw:.2f}, topdown_yaw_ignored={is_topdown})')
    adx=abs(float(dx))
    if adx>x_hard:
        hard.append('OUTSIDE_X_HARD_ROI')
        details.append(f'dx={float(dx):.3f}m exceeds hard_limit={x_hard:.3f}m by {adx-x_hard:.3f}m ({(adx/x_hard-1.0)*100:.1f}%)')
    elif adx>x_abs:
        relaxed.append('ABNORMAL_X_MOTION_SEMANTIC_RESCUE')
        details.append(f'dx={float(dx):.3f}m exceeds normal_limit={x_abs:.3f}m by {adx-x_abs:.3f}m ({(adx/x_abs-1.0)*100:.1f}%); within hard_limit={x_hard:.3f}m')
    elif adx>x_default:
        relaxed.append('SEMANTIC_EXTENDED_X_ROI')
    if abs(float(dy))>y_abs: hard.append('OUTSIDE_Y_ROI')
    if abs(float(dz))>z_abs: hard.append('OUTSIDE_Z_ROI')
    if 'DIAMETER_GATE' in reasons:
        if view_partial and float(o.diameter)<=float(t.stable_diameter)+float(mc.get('diameter_gate_m',0.36)):
            relaxed.append('TOPDOWN_CLIPPED_DIAMETER')
        else:
            (relaxed if partial_positive else hard).append('PARTIAL_RELAX_DIAMETER' if partial_positive else 'DIAMETER_GATE')
    if 'LENGTH_GATE' in reasons:
        if view_partial and float(o.length)<=float(t.stable_length)+float(mc.get('length_gate_m',0.38)):
            relaxed.append('TOPDOWN_CLIPPED_LENGTH')
        else:
            (relaxed if partial_positive else hard).append('PARTIAL_RELAX_LENGTH' if partial_positive else 'LENGTH_GATE')
    if hard:
        msg=('FIRST_LAYER_FOUNDATION_LOCK: ' if foundation_locked else '')+'semantic rank is unique but geometric safety validation failed: '+','.join(sorted(set(hard+relaxed)))
        if details: msg+='; '+'; '.join(details)
        return {
            'status':'UNCERTAIN','cost':float(cfg.get('semantic',{}).get('uncertain_pair_cost',8.0))+_pair_cost_from_resolved(t,o,resolved_center,cfg)[0],
            'confidence':0.0,'reason_codes':sorted(set(hard+relaxed)),
            'visibility_ratio':ratio,'observation_mode':getattr(o,'observation_mode','MULTI_VIEW'),'resolved_center':[float(x) for x in resolved_center],
            'reason':msg
        }
    base,resolved_components=_pair_cost_from_resolved(t,o,resolved_center,cfg)
    if relaxed:
        abnormal=any(str(x).startswith('ABNORMAL_') for x in relaxed)
        penalty=float(cfg.get('semantic',{}).get('abnormal_motion_penalty',0.55) if abnormal else cfg.get('semantic',{}).get('relaxed_pair_penalty',0.25))*len(set(relaxed))
        if abnormal:
            # Identity is accepted because semantic order/cardinality leaves no legal
            # alternative; the motion itself remains an explicit abnormal condition.
            conf=float(np.clip(float(cfg.get('semantic',{}).get('abnormal_motion_confidence',0.64))-0.05*min(base,2.0),0.50,0.72))
        else:
            conf=float(np.clip(0.82-(0.10*len(set(relaxed)))-0.10*min(base,1.0),0.55,0.88))
        msg=('FIRST_LAYER_FOUNDATION_LOCK: ' if foundation_locked else '')+'semantic order/cardinality lock + geometry validated with controlled relaxation: '+','.join(sorted(set(relaxed)))
        if details: msg+='; '+'; '.join(details)
        return {
            'status':'MATCHED','cost':base+penalty,'confidence':conf,'reason_codes':sorted(set(relaxed)),
            'visibility_ratio':ratio,'observation_mode':getattr(o,'observation_mode','MULTI_VIEW'),'resolved_center':[float(x) for x in resolved_center],'resolved_components':resolved_components,
            'reason':msg
        }
    skip_ref=max(float(mc.get('skip_history_cost',0.9)),1e-6)
    conf=float(np.clip(1.0-base/skip_ref,0.20,0.99))
    return {'status':'MATCHED','cost':base,'confidence':conf,'reason_codes':[],'visibility_ratio':ratio,'observation_mode':getattr(o,'observation_mode','MULTI_VIEW'),'resolved_center':[float(x) for x in resolved_center],'resolved_components':resolved_components,'reason':(('FIRST_LAYER_FOUNDATION_LOCK: ' if foundation_locked else '')+'semantic order/cardinality lock + geometry validation passed')}


def _hint_pair_assessment(o:Observation,h:NewCoilHint,cfg:dict):
    nc=cfg.get('new_coil',{})
    pos_tol=float(nc.get('position_tolerance_m',0.50))
    d_tol=float(nc.get('diameter_tolerance_m',0.25))
    l_tol=float(nc.get('length_tolerance_m',0.30))
    resolved=_topdown_resolved_center(o,h.target_center,h.diameter,h.length)
    dist=float(np.linalg.norm(np.asarray(resolved,dtype=float)-np.asarray(h.target_center,dtype=float)))
    top=getattr(o,'observation_mode','MULTI_VIEW')=='TOP_DOWN_Z'
    # Under-size is expected when the top projection is clipped by another coil.
    dD=max(0.0,float(o.diameter)-float(h.diameter)) if top else abs(float(o.diameter)-float(h.diameter))
    dL=max(0.0,float(o.length)-float(h.length)) if top else abs(float(o.length)-float(h.length))
    reasons=[]
    if dist>pos_tol: reasons.append('NEW_HINT_POSITION_GATE')
    if dD>d_tol*1.5: reasons.append('NEW_HINT_DIAMETER_GATE')
    if dL>l_tol*1.5: reasons.append('NEW_HINT_LENGTH_GATE')
    score=dist/max(pos_tol,1e-6)+0.20*dD/max(d_tol,1e-6)+0.20*dL/max(l_tol,1e-6)
    return {'accepted':not reasons,'cost':float(score),'distance_m':dist,'diameter_delta_m':dD,'length_delta_m':dL,'reasons':reasons,'resolved_center':[float(x) for x in resolved],'observation_mode':getattr(o,'observation_mode','MULTI_VIEW')}


def _assign_new_slots_batch(layer:int, inserts:list[Observation], tracks:list[TrackState], cfg:dict):
    """Assign legal semantic slots to already-selected NEW observations.

    This is a topology operation, not an identity classifier.  L1 grows inward
    from one/both truck edges; the normal middle free region is allowed.  L2 keeps
    the V1 append rule.
    """
    if not inserts:
        return {},[]
    inserts=sorted(inserts,key=lambda o:float(o.center[0]))
    lt=sorted([t for t in tracks if t.layer==layer],key=lambda t:t.slot)
    if layer==2:
        slots=[t.slot for t in lt]; start=max(slots)+1 if slots else 0
        max_l2=int(cfg.get('business',{}).get('layer2_max_slots',8))
        if start+len(inserts)>max_l2:
            return {},['LAYER2_CAPACITY_EXCEEDED']
        return {o.instance_id:(start+i,'layer2') for i,o in enumerate(inserts)},[]

    cap=int(cfg.get('business',{}).get('layer1_max_slots',10))
    occupied={t.slot for t in lt}
    if len(occupied)+len(inserts)>cap:
        return {},['LAYER1_CAPACITY_EXCEEDED']
    if any(t.cluster=='full_L1' for t in lt):
        return {},['LAYER1_ALREADY_FULL']
    left=[t for t in lt if t.cluster in ('left','full_L1')]
    right=[t for t in lt if t.cluster=='right']
    if not lt:
        mid=float(cfg.get('initialization',{}).get('truck_mid_x_m',7.5))
        if float(np.mean([o.center[0] for o in inserts]))<=mid:
            return {o.instance_id:(i,'left') for i,o in enumerate(inserts)},[]
        start=cap-len(inserts)
        return {o.instance_id:(start+i,'right') for i,o in enumerate(inserts)},[]
    if left and not right:
        start=max(t.slot for t in left)+1
        if start+len(inserts)>cap: return {},['LEFT_CLUSTER_NO_FREE_SLOT']
        return {o.instance_id:(start+i,'left') for i,o in enumerate(inserts)},[]
    if right and not left:
        start=min(t.slot for t in right)-len(inserts)
        if start<0: return {},['RIGHT_CLUSTER_NO_FREE_SLOT']
        return {o.instance_id:(start+i,'right') for i,o in enumerate(inserts)},[]
    if not left or not right:
        return {},['UNKNOWN_LAYER1_CLUSTER_TOPOLOGY']

    # Two-cluster case: sorted NEW observations can only extend the left inner
    # boundary, the right inner boundary, or both. Enumerate the split point.
    left_slot=max(t.slot for t in left); right_slot=min(t.slot for t in right)
    left_x=float(max(left,key=lambda t:t.slot).center[0]); right_x=float(min(right,key=lambda t:t.slot).center[0])
    best=None
    for p in range(len(inserts)+1):
        left_slots=list(range(left_slot+1,left_slot+1+p))
        rcount=len(inserts)-p
        right_slots=list(range(right_slot-rcount,right_slot))
        slots=left_slots+right_slots
        if len(slots)!=len(inserts) or any(s<0 or s>=cap or s in occupied for s in slots): continue
        if slots and len(set(slots))!=len(slots): continue
        if left_slots and right_slots and max(left_slots)>=min(right_slots): continue
        score=0.0
        for i,o in enumerate(inserts[:p]): score+=abs(float(o.center[0])-left_x)
        for o in inserts[p:]: score+=abs(float(o.center[0])-right_x)
        if best is None or score<best[0]: best=(score,p,slots)
    if best is None:
        return {},['MIDDLE_FREE_REGION_HAS_NO_LEGAL_NEW_SLOTS']
    _,p,slots=best
    out={}
    for i,o in enumerate(inserts): out[o.instance_id]=(slots[i],'left' if i<p else 'right')
    return out,[]


def _layer_semantic_hypotheses(layer:int,tracks:list[TrackState],observations:dict[int,Observation],hints:list[NewCoilHint],candidates:list[Candidate],cfg:dict):
    """Enumerate order-preserving, no-history-skip hypotheses for one layer."""
    import itertools
    H=sorted([t for t in tracks if t.layer==layer],key=lambda t:t.slot)
    O=sorted([o for o in observations.values() if o.layer_guess==layer],key=lambda o:float(o.center[0]))
    LH=sorted([h for h in hints if _hint_layer(h,cfg)==layer],key=lambda h:float(h.target_center[0]))
    m,n,k=len(H),len(O),len(LH)
    dbg={'layer':layer,'history_count':m,'observation_count':n,'expected_new_count':k,
         'history_order':[t.global_id for t in H],'observation_order':[o.instance_id for o in O],
         'hint_order':[h.hint_id for h in LH], 'mode':'SEMANTIC_ORDER_BIJECTION'}
    expected=m+k
    if n!=expected:
        code='ABNORMAL_CARDINALITY'
        dbg.update({'status':'ABNORMAL','reason_code':code,'expected_current_count':expected,
                    'message':f'layer {layer}: current observations={n}, expected history({m})+NEW({k})={expected}; zero-point hidden old coil is not a normal explanation'})
        return None,dbg,[code]
    cmap=_candidate_all_map(candidates)
    hypotheses=[]
    combos=[()] if k==0 else itertools.combinations(range(n),k)
    for comb in combos:
        ins_idx=set(comb); inserts=[O[j] for j in comb]
        if len(inserts)!=k: continue
        hint_assess=[]; good=True; score=0.0
        for o,h in zip(inserts,LH):
            a=_hint_pair_assessment(o,h,cfg); hint_assess.append({'instance_id':o.instance_id,'hint_id':h.hint_id,**a})
            if not a['accepted']:
                good=False; break
            score+=float(cfg.get('semantic',{}).get('new_hint_weight',0.60))*a['cost']
        if not good: continue
        old_obs=[o for j,o in enumerate(O) if j not in ins_idx]
        if len(old_obs)!=m: continue
        pairs=[]
        for t,o in zip(H,old_obs):
            cand=cmap.get((t.global_id,o.instance_id))
            if cand is None: good=False; break
            a=_semantic_pair_assessment(t,o,cand,cfg)
            pairs.append({'global_id':t.global_id,'instance_id':o.instance_id,**a})
            score+=a['cost']
        if not good: continue
        slot_map,slot_errors=_assign_new_slots_batch(layer,inserts,tracks,cfg)
        if slot_errors: continue
        hypotheses.append({'score':float(score),'insert_indices':list(comb),'inserts':inserts,'pairs':pairs,
                           'hint_assess':hint_assess,'slot_map':slot_map})
    if not hypotheses:
        code='ABNORMAL_NO_LEGAL_SEMANTIC_HYPOTHESIS'
        dbg.update({'status':'ABNORMAL','reason_code':code,'message':'no order-preserving hypothesis satisfies expected NEW hints and legal slot growth'})
        return None,dbg,[code]
    hypotheses.sort(key=lambda x:x['score'])
    best=hypotheses[0]; second=hypotheses[1]['score'] if len(hypotheses)>1 else None
    margin=None if second is None else float(second-best['score'])
    margin_min=float(cfg.get('semantic',{}).get('hypothesis_margin_min',0.20))
    dbg.update({'status':'OK','best_score':best['score'],'second_score':second,'margin':margin,
                'hypothesis_count':len(hypotheses),'best_insert_instances':[o.instance_id for o in best['inserts']],
                'best_pairs':best['pairs'],'hint_assess':best['hint_assess'],'new_slot_map':{str(k):v for k,v in best['slot_map'].items()}})
    if margin is not None and margin<margin_min:
        code='UNCERTAIN_SEMANTIC_HYPOTHESIS_MARGIN'
        dbg.update({'status':'UNCERTAIN','reason_code':code,'message':f'best/second semantic hypothesis margin {margin:.3f} < {margin_min:.3f}'})
        return best,dbg,[code]
    return best,dbg,[]



def _second_layer_support_hypothesis(tracks:list[TrackState], observations:dict[int,Observation], hints:list[NewCoilHint], candidates:list[Candidate], cfg:dict):
    """Deterministic L2 association from support topology.

    In second-layer loading mode, upper identity is its support valley. Slot k means
    SUPPORTED_BY(L1[k], L1[k+1]). An upper coil is not allowed to silently hop to a
    different valley; ambiguity/physical inconsistency is surfaced instead of causing
    a global-ID shift.
    """
    dc=cfg.get('double_layer',{}); req=int(dc.get('layer1_required_slots',10))
    L1=sorted([t for t in tracks if t.layer==1],key=lambda t:t.slot)
    H=sorted([t for t in tracks if t.layer==2],key=lambda t:t.slot)
    O=sorted([o for o in observations.values() if o.layer_guess==2],key=lambda o:float(o.center[0]))
    LH=sorted([h for h in hints if _hint_layer(h,cfg)==2],key=lambda h:float(h.target_center[0]))
    dbg={'layer':2,'mode':'SECOND_LAYER_SUPPORT_TOPOLOGY','history_count':len(H),'observation_count':len(O),'expected_new_count':len(LH),
         'history_slots':[int(t.slot) for t in H],'observation_order':[int(o.instance_id) for o in O]}
    if len(L1)!=req or {int(t.slot) for t in L1}!=set(range(req)):
        code='ABNORMAL_SECOND_LAYER_WITHOUT_FULL_L1'
        dbg.update({'status':'ABNORMAL','reason_code':code,'message':f'second-layer signal requires locked full L1 slots 0..{req-1}; found {len(L1)} tracks'})
        return None,dbg,[code]
    max_coils=int(cfg.get('business',{}).get('layer2_max_coils',8))
    if len(H)+len(LH)>max_coils:
        code='ABNORMAL_LAYER2_CAPACITY'
        dbg.update({'status':'ABNORMAL','reason_code':code,'message':f'upper count history({len(H)})+NEW({len(LH)}) exceeds {max_coils}'})
        return None,dbg,[code]
    if len(O)!=len(H)+len(LH):
        code='ABNORMAL_CARDINALITY'
        dbg.update({'status':'ABNORMAL','reason_code':code,'message':f'L2 observations={len(O)} expected history({len(H)})+NEW({len(LH)}); zero-point occlusion is not allowed'})
        return None,dbg,[code]
    normal=float(dc.get('support_slot_max_center_error_m',0.42)); hard=float(dc.get('support_slot_hard_center_error_m',0.65))
    obs_slot={}; support_audit=[]
    for o in O:
        slot,err,ref=_nearest_support_slot(tracks,o.center)
        support_audit.append({'instance_id':int(o.instance_id),'support_slot':slot,'support_pair':None if slot is None else [slot,slot+1],'xy_error_m':float(err),'support_xy':None if ref is None else [float(ref[0]),float(ref[1])]})
        if slot is None or err>hard:
            code='ABNORMAL_LAYER2_OFF_SUPPORT'
            dbg.update({'status':'ABNORMAL','reason_code':code,'support_audit':support_audit,'message':f'upper instance {o.instance_id} is not inside any legal support valley; nearest error={err:.3f}m hard={hard:.3f}m'})
            return None,dbg,[code]
        if slot in obs_slot:
            code='ABNORMAL_LAYER2_SUPPORT_COLLISION'
            dbg.update({'status':'ABNORMAL','reason_code':code,'support_audit':support_audit,'message':f'multiple current upper instances resolve to support slot {slot}'})
            return None,dbg,[code]
        obs_slot[slot]=o
    hist_slots={int(t.slot) for t in H}
    current_slots=set(obs_slot)
    missing=sorted(hist_slots-current_slots); extras=sorted(current_slots-hist_slots)
    if missing:
        code='ABNORMAL_LAYER2_SUPPORT_SLOT_MISSING'
        dbg.update({'status':'ABNORMAL','reason_code':code,'support_audit':support_audit,'missing_history_slots':missing,'message':'historical upper support slot has no positive current instance; do not shift later IDs'})
        return None,dbg,[code]
    if len(extras)!=len(LH):
        code='ABNORMAL_LAYER2_NEW_SUPPORT_COUNT'
        dbg.update({'status':'ABNORMAL','reason_code':code,'support_audit':support_audit,'extra_support_slots':extras,'message':f'new support slots {extras} do not match robot NEW count {len(LH)}'})
        return None,dbg,[code]
    cmap=_candidate_all_map(candidates); pairs=[]; score=0.0; codes=[]
    for t in H:
        o=obs_slot[int(t.slot)]
        cand=cmap.get((t.global_id,o.instance_id))
        if cand is None:
            code='ABNORMAL_NO_LAYER2_CANDIDATE'
            dbg.update({'status':'ABNORMAL','reason_code':code,'support_audit':support_audit,'message':f'no candidate for historical upper gid {t.global_id} at its support slot {t.slot}'})
            return None,dbg,[code]
        a=_semantic_pair_assessment(t,o,cand,cfg)
        err=next(x['xy_error_m'] for x in support_audit if x['instance_id']==o.instance_id)
        rc=list(a['reason_codes'])
        reason=a['reason']+f'; SUPPORT_SLOT={t.slot} supported_by L1[{t.slot},{t.slot+1}] err={err:.3f}m'
        if err>normal and err<=hard:
            rc.append('ABNORMAL_LAYER2_SUPPORT_OFFSET'); codes.append('ABNORMAL_LAYER2_SUPPORT_OFFSET')
            reason+=f' > normal support tolerance {normal:.3f}m'
            if a['status']=='MATCHED': a['confidence']=min(float(a['confidence']),0.68); a['cost']=float(a['cost'])+0.45
        pairs.append({'global_id':t.global_id,'instance_id':o.instance_id,**a,'reason_codes':sorted(set(rc)),'reason':reason,'support_slot':int(t.slot)})
        score+=float(a['cost'])
    hint_assess=[]; slot_map={}
    # Robot hint does not carry identity, but it should land in the same discrete support valley as the extra observation.
    remaining_hints=list(LH)
    for slot in extras:
        o=obs_slot[slot]
        if not remaining_hints:
            break
        ranked=[]
        for h in remaining_hints:
            ha=_hint_pair_assessment(o,h,cfg)
            hslot,herr,_=_nearest_support_slot(tracks,h.target_center)
            ranked.append((0 if hslot==slot else 1,float(ha['cost']),h,ha,hslot,herr))
        ranked.sort(key=lambda x:(x[0],x[1])); _,_,h,ha,hslot,herr=ranked[0]
        if not ha['accepted'] or hslot!=slot:
            code='ABNORMAL_NEW_HINT_SUPPORT_MISMATCH'
            dbg.update({'status':'ABNORMAL','reason_code':code,'support_audit':support_audit,'message':f'NEW observation at support {slot} is incompatible with robot target support {hslot}'})
            return None,dbg,[code]
        remaining_hints.remove(h)
        hint_assess.append({'instance_id':o.instance_id,'hint_id':h.hint_id,'support_slot':slot,'hint_support_error_m':float(herr),**ha})
        slot_map[o.instance_id]=(slot,'layer2')
        score+=float(cfg.get('semantic',{}).get('new_hint_weight',0.60))*float(ha['cost'])
    best={'score':float(score),'inserts':[obs_slot[s] for s in extras],'pairs':pairs,'hint_assess':hint_assess,'slot_map':slot_map,'support_audit':support_audit}
    dbg.update({'status':'OK','best_score':float(score),'support_audit':support_audit,'historical_support_slots':sorted(hist_slots),'new_support_slots':extras,
                'new_slot_map':{str(k):v for k,v in slot_map.items()},'best_pairs':pairs,'hint_assess':hint_assess})
    return best,dbg,codes


def apply_topdown_semantic_layer_assignment(tracks:list[TrackState], observations:dict[int,Observation], hints:list[NewCoilHint], cfg:dict):
    """Resolve layer membership for a TOP_DOWN_Z frame before candidate gating.

    A lower-layer coil can be strongly clipped by an upper coil in a -Z projection.
    Its single-instance bbox/radius estimate may then put its apparent center above the
    layer split.  Treating that raw layer guess as a hard gate destroys cardinality
    before the semantic solver gets a chance to use history.

    In normal operation layer is immutable for historical tracks and robot hints state
    the expected layer of NEW coils.  Therefore, when total cardinality is consistent,
    the expected number of observations per layer is known.  For a top-camera frame we
    assign exactly that many observations to each layer using robust *top surface Z*:
    upper-layer cylinders have the highest top surfaces even when their visible width
    is clipped.  This is a semantic cardinality constraint plus a direct view-space
    measurement, not a GT label.

    The function mutates Observation.layer_guess intentionally *before* candidates are
    built, so candidate gating and the semantic solver see the same resolved layer.
    """
    obs=list(observations.values())
    audit={
        'applied':False, 'mode':'GEOMETRIC_LAYER_GUESS',
        'topdown_fraction':0.0, 'expected_by_layer':{}, 'before':{}, 'after':{},
    }
    if not obs:
        return audit
    frac=sum(getattr(o,'observation_mode','MULTI_VIEW')=='TOP_DOWN_Z' for o in obs)/len(obs)
    audit['topdown_fraction']=float(frac)
    audit['before']={str(o.instance_id):int(o.layer_guess) for o in obs}
    tc=cfg.get('topdown_detection',{})
    min_frac=float(tc.get('semantic_layer_assignment_fraction',tc.get('frame_consensus_fraction',0.55)))
    if frac < min_frac:
        audit['after']=dict(audit['before'])
        return audit

    expected={}
    for t in tracks:
        expected[int(t.layer)]=expected.get(int(t.layer),0)+1
    for h in hints:
        layer=int(_hint_layer(h,cfg))
        expected[layer]=expected.get(layer,0)+1
    audit['expected_by_layer']={str(k):int(v) for k,v in sorted(expected.items())}
    if sum(expected.values()) != len(obs):
        # Do not manufacture a layer partition when frame cardinality itself is abnormal.
        audit['mode']='TOPDOWN_SEMANTIC_LAYER_ASSIGNMENT_SKIPPED_CARDINALITY'
        audit['reason']=f'current={len(obs)} expected_total={sum(expected.values())}'
        audit['after']=dict(audit['before'])
        return audit

    # Current V1 business model has layers 1/2. Keep the implementation deterministic
    # and conservative if a future configuration introduces more layers.
    layers=sorted(expected)
    if not layers:
        audit['after']=dict(audit['before'])
        return audit
    if len(layers)==1:
        only=layers[0]
        for o in obs: o.layer_guess=only
    elif set(layers).issubset({1,2}):
        n2=int(expected.get(2,0))
        ranked=sorted(obs,key=lambda o:(float(getattr(o,'top_z',o.bbox_max[2])),float(o.center[2]),int(o.instance_id)),reverse=True)
        layer2_ids={o.instance_id for o in ranked[:n2]}
        for o in obs:
            o.layer_guess=2 if o.instance_id in layer2_ids else 1
        audit['top_z_rank']=[{'instance_id':int(o.instance_id),'top_z':float(getattr(o,'top_z',o.bbox_max[2])),'assigned_layer':int(o.layer_guess)} for o in ranked]
    else:
        # Future multi-layer fallback: split by top_z according to required counts from
        # highest layer downward. This remains cardinality-driven and deterministic.
        ranked=sorted(obs,key=lambda o:(float(getattr(o,'top_z',o.bbox_max[2])),int(o.instance_id)),reverse=True)
        remaining=list(ranked)
        for layer in sorted(layers,reverse=True):
            take=int(expected[layer])
            chosen=remaining[:take]; remaining=remaining[take:]
            for o in chosen: o.layer_guess=layer
    audit['applied']=True
    audit['mode']='TOPDOWN_SEMANTIC_CARDINALITY_PLUS_TOP_Z'
    audit['after']={str(o.instance_id):int(o.layer_guess) for o in obs}
    audit['changed_instances']=[int(o.instance_id) for o in obs if audit['before'].get(str(o.instance_id))!=int(o.layer_guess)]
    return audit

def semantic_constrained_match(tracks:list[TrackState], observations:dict[int,Observation], hints:list[NewCoilHint], candidates:list[Candidate], cfg:dict):
    """V0.6 production matcher skeleton.

    Hard business model:
      * no historical removal in normal operation;
      * same-layer semantic order cannot reverse;
      * a historical coil called OCCLUDED must still have a non-empty current instance;
      * expected NEW count comes from robot hints;
      * when counts are consistent, history/current association is an ordered bijection
        after selecting the hinted NEW observations. SKIP_HISTORY is not in the normal
        solution space.

    Geometry validates the semantic mapping. If geometry violates a safety envelope,
    the affected identity becomes UNCERTAIN instead of shifting later IDs.
    """
    decisions=[]; actions=[]; layer_debug={}; codes=[]
    layers=sorted(set([t.layer for t in tracks]+[o.layer_guess for o in observations.values()]+[_hint_layer(h,cfg) for h in hints]))
    for layer in layers:
        H=sorted([t for t in tracks if t.layer==layer],key=lambda t:t.slot)
        O=sorted([o for o in observations.values() if o.layer_guess==layer],key=lambda o:float(o.center[0]))
        if layer==2 and _double_layer_active(cfg):
            best,dbg,layer_codes=_second_layer_support_hypothesis(tracks,observations,hints,candidates,cfg)
        else:
            best,dbg,layer_codes=_layer_semantic_hypotheses(layer,tracks,observations,hints,candidates,cfg)
        layer_debug[str(layer)]=dbg; codes.extend(layer_codes)
        if best is None or dbg.get('status')=='UNCERTAIN':
            code=dbg.get('reason_code','ABNORMAL_SEMANTIC_LAYER')
            # Do not invent a zero-point occlusion. Preserve historical state and expose
            # every current instance as unassigned/uncertain for this affected layer.
            for t in H:
                decisions.append(Decision(None,t.global_id,'UNCERTAIN',0.0,t.layer,t.slot,t.cluster,code+': historical identity not safely commit-able'))
                actions.append({'action':'HOLD_HISTORY_UNCERTAIN','global_id':t.global_id,'instance_id':None,'cost':0.0,'reason_code':code})
            for o in O:
                decisions.append(Decision(o.instance_id,None,'UNCERTAIN',0.0,o.layer_guess,None,None,code+': current instance not safely assigned'))
                actions.append({'action':'CURRENT_UNCERTAIN','global_id':None,'instance_id':o.instance_id,'cost':0.0,'reason_code':code})
            continue
        # Commit structurally unique mapping; individual geometry can still be uncertain.
        pair_by_iid={p['instance_id']:p for p in best['pairs']}
        for p in best['pairs']:
            t=next(t for t in H if t.global_id==p['global_id'])
            if p.get('resolved_center') is not None:
                observations[p['instance_id']].center=np.asarray(p['resolved_center'],dtype=float)
            if p['status']=='MATCHED':
                decisions.append(Decision(p['instance_id'],t.global_id,'MATCHED',float(p['confidence']),t.layer,t.slot,t.cluster,p['reason'],float(p['cost'])))
                actions.append({'action':'SEMANTIC_MATCH','global_id':t.global_id,'instance_id':p['instance_id'],'cost':float(p['cost']),'reason_codes':p['reason_codes']})
                # Abnormal motion can coexist with a safe identity. Surface it at frame
                # level instead of converting the identity to UNCERTAIN.
                for rc in p['reason_codes']:
                    if rc.startswith('ABNORMAL_') and rc not in codes: codes.append(rc)
            else:
                # Candidate current instance is recorded in the decision/debug, but commit_step
                # will NOT inherit the global ID; it emits a historical proxy + neutral current
                # observation so wrong-ID precision remains protected.
                decisions.append(Decision(p['instance_id'],t.global_id,'UNCERTAIN',0.0,t.layer,t.slot,t.cluster,p['reason'],float(p['cost'])))
                actions.append({'action':'SEMANTIC_PAIR_UNCERTAIN','global_id':t.global_id,'instance_id':p['instance_id'],'cost':float(p['cost']),'reason_codes':p['reason_codes']})
                codes.extend([c for c in p['reason_codes'] if c not in codes])
        for o in best['inserts']:
            slot,cluster=best['slot_map'][o.instance_id]
            ha=next(x for x in best['hint_assess'] if x['instance_id']==o.instance_id)
            if ha.get('resolved_center') is not None:
                observations[o.instance_id].center=np.asarray(ha['resolved_center'],dtype=float)
            new_reason=('robot NEW hint + support valley topology + full locked L1 foundation' if layer==2 and _double_layer_active(cfg)
                        else 'robot NEW hint + legal inward slot growth + semantic cardinality')
            decisions.append(Decision(o.instance_id,None,'NEW',0.95,layer,int(slot),cluster,
                new_reason,None,ha['hint_id']))
            actions.append({'action':'SEMANTIC_NEW','global_id':None,'instance_id':o.instance_id,'cost':float(ha['cost']),'hint_id':ha['hint_id'],'slot':int(slot),'cluster':cluster})
    # De-duplicate codes while preserving order.
    uniq=[]
    for c in codes:
        if c not in uniq: uniq.append(c)
    frame_status='ABNORMAL' if any(c.startswith('ABNORMAL_') for c in uniq) else ('UNCERTAIN' if any(d.state=='UNCERTAIN' for d in decisions) else 'OK')
    return {'decisions':decisions,'actions':actions,'layers':layer_debug,'reason_codes':uniq,'frame_status':frame_status}
