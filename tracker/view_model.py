from __future__ import annotations

import numpy as np


def topdown_z_visible_mask(points: np.ndarray, cfg: dict | None = None) -> np.ndarray:
    """Return points visible from a camera looking along world -Z.

    This is a point-cloud z-buffer rather than a naive ``z > center`` crop. XY is
    discretized into small cells; for each sight line only points close to the
    highest observed Z are retained. The finite surface thickness keeps a realistic
    local patch instead of one mathematical point per pixel.
    """
    p=np.asarray(points,dtype=float)
    if len(p)==0:
        return np.zeros(0,dtype=bool)
    vc=(cfg or {}).get('topdown_view',{}) if isinstance(cfg,dict) else {}
    cell=float(vc.get('xy_cell_m',0.050))
    thickness=float(vc.get('surface_thickness_m',0.055))
    x0=float(np.min(p[:,0])); y0=float(np.min(p[:,1]))
    ix=np.floor((p[:,0]-x0)/max(cell,1e-6)).astype(np.int64)
    iy=np.floor((p[:,1]-y0)/max(cell,1e-6)).astype(np.int64)
    # Pair key without assumptions about absolute scene origin.
    key=(ix<<32) ^ (iy & np.int64(0xffffffff))
    order=np.argsort(key,kind='mergesort'); sk=key[order]
    starts=np.r_[0,np.flatnonzero(np.diff(sk))+1]
    unique=sk[starts]
    maxz=np.maximum.reduceat(p[order,2],starts)
    pos=np.searchsorted(unique,key)
    return p[:,2] >= (maxz[pos]-thickness)


def apply_topdown_z_to_scene(scene: list[dict], cfg: dict | None = None, *, ensure_positive: bool = True):
    """Apply a global -Z visibility projection to a list of scene objects.

    The z-buffer is global, so upper-layer objects can occlude lower-layer points in
    the same projected XY cells. In the current business model every historical coil
    must retain some positive observation; if discretization/adversarial placement
    removes an object completely, a small set of its highest-Z points is restored and
    a warning is returned. This is explicit synthetic-data protection, not a matcher
    assumption that zero-point occlusion is acceptable.
    """
    if not scene:
        return scene,[]
    chunks=[]; owner=[]
    for idx,o in enumerate(scene):
        p=np.asarray(o.get('points',[]),dtype=np.float32)
        if len(p):
            chunks.append(p); owner.append(np.full(len(p),idx,dtype=np.int32))
    if not chunks:
        return scene,[]
    xyz=np.vstack(chunks); oid=np.concatenate(owner)
    mask=topdown_z_visible_mask(xyz,cfg)
    vc=(cfg or {}).get('topdown_view',{}) if isinstance(cfg,dict) else {}
    min_points=int(vc.get('min_visible_points_per_instance',80))
    min_ratio=float(vc.get('min_visible_ratio_per_instance',0.025))
    warnings=[]
    out=[]
    cursor=0
    for idx,o in enumerate(scene):
        original=np.asarray(o.get('points',[]),dtype=np.float32)
        n=len(original)
        local_mask=mask[cursor:cursor+n] if n else np.zeros(0,dtype=bool)
        visible=original[local_mask]
        cursor+=n
        target=min(n,max(min_points,int(round(n*min_ratio)))) if n else 0
        if ensure_positive and n and len(visible)<target:
            # Restore the physically most top-facing samples of this object. This is
            # only a guard against a synthetic fully-invisible object, which the user
            # explicitly said is outside the current operating model.
            need=target-len(visible)
            already=set(np.flatnonzero(local_mask).tolist())
            candidates=np.argsort(original[:,2])[::-1]
            add_idx=[int(i) for i in candidates if int(i) not in already][:need]
            if add_idx:
                visible=np.vstack([visible,original[add_idx]]) if len(visible) else original[add_idx].copy()
            warnings.append(f"ID{o.get('global_id')} top-down projection would leave too few points; restored top-facing samples to preserve positive visibility")
        q=dict(o); q['points']=visible.astype(np.float32); q['view_mode']='TOP_DOWN_Z'; out.append(q)
    return out,warnings
