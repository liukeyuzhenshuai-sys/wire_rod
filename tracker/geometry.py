from __future__ import annotations
import math
import numpy as np
from .models import Observation


def _robust_range(v, q=0.01):
    lo,hi=np.quantile(v,[q,1-q]); return float(lo),float(hi)


def _estimate_yaw_xy(points):
    # Conservative yaw estimator. Search for the horizontal normal that minimizes
    # robust cross-axis width, then use its orthogonal direction as the coil axis.
    # Yaw remains a weak matching feature because partial surfaces can bias it.
    angles=np.linspace(-12.0,12.0,97)
    widths=[]
    xy=points[:,:2]
    for th in angles:
        r=math.radians(th)
        # axis ~ [sin(yaw), cos(yaw)], normal ~ [cos(yaw), -sin(yaw)]
        n=np.array([math.cos(r),-math.sin(r)])
        u=xy@n
        lo,hi=np.quantile(u,[.01,.99]); widths.append(hi-lo)
    return float(angles[int(np.argmin(widths))])


def _topdown_assessment(points:np.ndarray, diameter:float, cfg:dict):
    """Infer whether an instance looks like a -Z top-only projection.

    The matcher is intentionally not given synthetic GT/view labels. Detection uses
    only XYZ. A true top-only cylinder keeps most of its XY footprint but its robust
    Z span is roughly half a diameter rather than a full diameter. This is a strong
    and interpretable cue for the generated/production-like top-camera mode.
    """
    tc=cfg.get('topdown_detection',{})
    if not bool(tc.get('enabled',True)) or len(points)<40 or diameter<=1e-5:
        return False,0.0,0.0
    q=float(tc.get('z_span_quantile',0.01))
    zlo,zhi=_robust_range(points[:,2],q)
    ratio=float((zhi-zlo)/max(diameter,1e-6))
    thr=float(tc.get('z_span_to_diameter_max',0.72))
    strong=float(tc.get('z_span_to_diameter_strong',0.58))
    # Soft likelihood is useful for debugging and future calibration.
    if ratio>=thr:
        likelihood=0.0
    elif ratio<=strong:
        likelihood=1.0
    else:
        likelihood=float((thr-ratio)/max(thr-strong,1e-6))
    return ratio<=thr,likelihood,ratio


def observe_instance(instance_id:int, points:np.ndarray, cfg:dict, force_topdown:bool=False)->Observation:
    if len(points)<20: raise ValueError(f'instance {instance_id} has too few points')
    q_center=float(cfg['geometry'].get('center_quantile',0.005))
    q_size=float(cfg['geometry'].get('size_quantile',0.01))
    lo=np.quantile(points,q_center,axis=0); hi=np.quantile(points,1-q_center,axis=0)
    center=(lo+hi)/2.0
    bmin=points.min(axis=0); bmax=points.max(axis=0)
    yaw=_estimate_yaw_xy(points)
    r=math.radians(yaw)
    axis=np.array([math.sin(r),math.cos(r),0.0])
    normal=np.array([math.cos(r),-math.sin(r),0.0])
    s=points@axis; u=points@normal
    slo,shi=_robust_range(s,q_size); ulo,uhi=_robust_range(u,q_size)
    length=max(0.0,shi-slo); diameter=max(0.0,uhi-ulo)

    topdown,top_likelihood,z_span_ratio=_topdown_assessment(points,diameter,cfg)
    if force_topdown:
        topdown=True; top_likelihood=max(top_likelihood,0.85)
    center_method='ROBUST_BBOX'
    if topdown:
        # For a top camera the bbox midpoint in Z lies above the true cylinder axis.
        # Horizontal footprint still estimates D well, so recover the geometric center
        # from the top surface: z_center ~= z_top - D/2. This is the critical adaptation
        # that keeps layer and Z-motion semantics stable across view-mode changes.
        tc=cfg.get('topdown_detection',{})
        top_q=float(tc.get('top_z_quantile',0.995))
        z_top=float(np.quantile(points[:,2],top_q))
        z_from_top=z_top-diameter/2.0
        # Keep correction bounded relative to raw estimate to avoid a false detector
        # causing an implausible jump.
        max_corr=float(tc.get('max_z_center_correction_m',0.55))
        raw_z=float(center[2])
        center[2]=raw_z+float(np.clip(z_from_top-raw_z,-max_corr,max_corr))
        center_method='TOP_SURFACE_MINUS_RADIUS'

    dmin,dmax=cfg['geometry']['diameter_range_m']; lmin,lmax=cfg['geometry']['length_range_m']
    point_score=min(1.0,math.log10(max(len(points),10))/4.0)
    dscore=max(0.0,1.0-abs(np.clip(diameter,dmin,dmax)-diameter)/0.25)
    lscore=max(0.0,1.0-abs(np.clip(length,lmin,lmax)-length)/0.30)
    # side/top clipping asymmetry proxy only in XY; top-only view is expected to be
    # vertically asymmetric and should not be punished for that.
    med=np.median(points,axis=0)
    asym=float(np.linalg.norm((med-center)[:2]))
    asym_score=max(0.0,1.0-asym/0.25)
    quality=float(np.clip(.35*point_score+.25*dscore+.25*lscore+.15*asym_score,0,1))
    if topdown:
        quality=float(np.clip(quality*float(cfg.get('topdown_detection',{}).get('quality_factor',0.96)),0,1))

    # Layer split uses corrected geometric Z when top-only view is detected.
    layer_guess=2 if center[2]>=float(cfg['geometry'].get('layer2_center_z_min_m',1.15)) else 1
    return Observation(
        int(instance_id),center.astype(float),int(len(points)),float(diameter),float(length),yaw,quality,layer_guess,
        bmin.astype(float),bmax.astype(float),float(np.ptp(points[:,0])),float(np.ptp(points[:,1])),
        'TOP_DOWN_Z' if topdown else 'MULTI_VIEW',float(top_likelihood),float(z_span_ratio),center_method,
        float(np.quantile(points[:,2],float(cfg.get('topdown_detection',{}).get('top_z_quantile',0.995)))),
        float(slo),float(shi),float(ulo),float(uhi)
    )


def observe_frame(instances:dict[int,np.ndarray],cfg:dict):
    out={iid:observe_instance(iid,p,cfg) for iid,p in instances.items()}
    tc=cfg.get('topdown_detection',{})
    # Frame-level consensus handles lower-layer coils whose projected width is
    # strongly clipped by upper-layer coils: their per-instance z-span/diameter
    # ratio can look non-topdown even though the whole acquisition is from above.
    if bool(tc.get('frame_consensus_enabled',True)) and len(out)>=2:
        frac=sum(o.observation_mode=='TOP_DOWN_Z' for o in out.values())/len(out)
        if frac>=float(tc.get('frame_consensus_fraction',0.55)):
            for iid,o in list(out.items()):
                if o.observation_mode!='TOP_DOWN_Z':
                    out[iid]=observe_instance(iid,instances[iid],cfg,force_topdown=True)
    return out
