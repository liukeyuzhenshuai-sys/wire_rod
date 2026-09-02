from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import io
import json
import math
import re
import zipfile
import numpy as np

from .models import TrackState, NewCoilHint, Observation
from .matching import infer_new_slot
from .view_model import apply_topdown_z_to_scene

PALETTE=np.array([
[230,70,70],[70,180,240],[70,210,130],[245,190,50],[180,95,235],[40,200,205],[245,120,45],[130,210,65],[235,90,180],[100,130,245],
[215,150,75],[75,210,195],[220,105,120],[135,95,225],[100,200,100],[235,155,210],[85,165,230],[210,210,75],[165,120,85],[90,215,160],
[245,95,85],[95,195,245],[110,225,135],[250,200,90],[195,115,245],[75,220,220],[250,135,70],[150,220,95],[245,115,195],[120,145,250],
[225,165,90],[90,220,205],[230,125,140],[155,115,235],[120,215,120],[245,175,220],[105,180,240],[220,220,100],[180,135,100],[110,225,175]],dtype=np.uint8)


def _num(v, default=0.0):
    try: return float(v)
    except (TypeError,ValueError): return default


def _rot_xy(points: np.ndarray, deg: float) -> np.ndarray:
    if abs(deg)<1e-12: return points.copy()
    r=math.radians(deg); c,s=math.cos(r),math.sin(r)
    q=points.copy(); x=q[:,0].copy(); y=q[:,1].copy()
    q[:,0]=x*c-y*s; q[:,1]=x*s+y*c
    return q


def _surface_perturb_local(p: np.ndarray, mode: str, rng: np.random.Generator) -> np.ndarray:
    if mode not in ('roll_mild','roll_heavy'): return p
    amp=0.015 if mode=='roll_mild' else 0.030
    keep=0.93 if mode=='roll_mild' else 0.82
    dup=0.04 if mode=='roll_mild' else 0.08
    q=p.copy(); x,z=q[:,0],q[:,2]; rad=np.sqrt(x*x+z*z)+1e-6; th=np.arctan2(z,x)
    ph1,ph2=rng.uniform(-math.pi,math.pi,2)
    dr=amp*(0.55*np.sin(3*th+ph1)+0.30*np.sin(7*th+ph2)+0.15*rng.normal(size=len(q)))
    q[:,0]+=dr*x/rad; q[:,2]+=dr*z/rad
    q+=rng.normal(0,0.010 if mode=='roll_mild' else 0.014,size=q.shape)
    q=q[rng.random(len(q))<keep]
    if len(q) and dup>0:
        m=int(len(q)*dup)
        if m:
            q=np.vstack([q,q[rng.integers(0,len(q),m)]+rng.normal(0,0.008,size=(m,3))])
    return q.astype(np.float32)


def _edge_occlude_local(p: np.ndarray, side: str, severity: float, rng: np.random.Generator) -> np.ndarray:
    """Occlude only local left/right edge regions and never remove all points.

    The coil local X direction is the cross-axis horizontal direction.  The central
    band is explicitly protected so the editor cannot create the physically wrong
    'hole in the middle' failure from the old generator.
    """
    if side in (None,'','none') or severity<=0: return p
    if len(p)<30: return p
    severity=float(np.clip(severity,0.05,0.95))
    x=p[:,0]; xmin,xmax=np.quantile(x,[.01,.99]); span=max(float(xmax-xmin),1e-4)
    width=min(0.42,0.10+0.40*severity)*span
    mask=np.ones(len(p),dtype=bool)
    if side in ('left','both'):
        zone=x < xmin+width
        grad=np.clip((x-xmin)/(width+1e-6),0,1)
        survive=(0.07+0.82*grad)
        toss=rng.random(len(p))<survive
        mask[zone]&=toss[zone]
    if side in ('right','both'):
        zone=x > xmax-width
        grad=np.clip((xmax-x)/(width+1e-6),0,1)
        survive=(0.07+0.82*grad)
        toss=rng.random(len(p))<survive
        mask[zone]&=toss[zone]
    out=p[mask]
    # Protect the center/top-observable region and enforce a non-empty partial observation.
    central=(x>xmin+.34*span)&(x<xmin+.66*span)
    central_pts=p[central]
    if len(central_pts):
        out_c=np.sum((out[:,0]>xmin+.34*span)&(out[:,0]<xmin+.66*span)) if len(out) else 0
        target=max(30,int(0.72*len(central_pts)))
        if out_c<target:
            add=central_pts[rng.choice(len(central_pts),min(target,len(central_pts)),replace=False)]
            out=np.vstack([out,add]) if len(out) else add.copy()
    min_keep=max(120,int(0.10*len(p)))
    if len(out)<min_keep:
        # Restore points closest to the local center rather than inventing a middle hole.
        idx=np.argsort(np.abs(x))[:min(min_keep,len(p))]
        out=p[idx]
    return out.astype(np.float32)


def _scale_proto(proto: np.ndarray, diameter: float, length: float) -> np.ndarray:
    p=proto.astype(np.float32).copy()
    xlo,xhi=np.quantile(p[:,0],[.01,.99]); zlo,zhi=np.quantile(p[:,2],[.01,.99]); ylo,yhi=np.quantile(p[:,1],[.01,.99])
    d0=max(float((xhi-xlo+zhi-zlo)/2.0),0.05); l0=max(float(yhi-ylo),0.05)
    p[:,0]*=float(diameter)/d0; p[:,2]*=float(diameter)/d0; p[:,1]*=float(length)/l0
    # recenter robustly; scene center represents the coil geometric center.
    lo=np.quantile(p,.01,axis=0); hi=np.quantile(p,.99,axis=0); p-=(lo+hi)/2.0
    return p.astype(np.float32)


def _pose_local(p: np.ndarray, center, yaw_deg: float) -> np.ndarray:
    q=_rot_xy(p,yaw_deg); q+=np.asarray(center,dtype=np.float32); return q.astype(np.float32)


def write_scene_ply(path: Path, objects: list[dict]) -> int:
    chunks=[]
    for o in objects:
        p=np.asarray(o['points'],dtype=np.float32)
        if not len(p): continue
        n=len(p); gid=int(o['global_id']); iid=int(o['instance_id']); layer=int(o['layer']); slot=int(o['slot'])
        vis={'VISIBLE':1,'PARTIAL_VISIBLE':2,'OCCLUDED':3,'UNOBSERVED':4,'VIEW_PARTIAL':5}.get(o.get('visibility','VISIBLE'),0)
        dt=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('global_id','<i4'),('instance_id','<i4'),('layer','u1'),('slot','<i2'),('visibility','u1')])
        a=np.empty(n,dtype=dt); a['x']=p[:,0];a['y']=p[:,1];a['z']=p[:,2]
        col=PALETTE[gid%len(PALETTE)];a['red']=col[0];a['green']=col[1];a['blue']=col[2]
        a['global_id']=gid;a['instance_id']=iid;a['layer']=layer;a['slot']=slot;a['visibility']=vis
        chunks.append(a)
    allv=np.concatenate(chunks) if chunks else np.empty(0,dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('global_id','<i4'),('instance_id','<i4'),('layer','u1'),('slot','<i2'),('visibility','u1')]))
    hdr=('ply\nformat binary_little_endian 1.0\n'
         'comment drag_scene_editor_v0_7_topdown_supported\n'
         'comment global_id_and_rgb_are_ground_truth_visualization_only_do_not_use_for_matching\n'
         'comment occlusion_requires_nonzero_partial_points_and_removes_only_edge_regions\n'
         f'element vertex {len(allv)}\n'
         'property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n'
         'property int global_id\nproperty int instance_id\nproperty uchar layer\nproperty short slot\nproperty uchar visibility\nend_header\n')
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('wb') as f: f.write(hdr.encode('ascii')); allv.tofile(f)
    return int(len(allv))


class InteractiveSceneEditor:
    def __init__(self, root: Path):
        self.root=Path(root)
        z=np.load(self.root/'assets'/'prototype_points.npz')
        self.protos={k:z[k].astype(np.float32) for k in z.files}
        self.source_to_key={}
        with open(self.root/'assets'/'prototype_map.csv','r',encoding='utf-8-sig',newline='') as f:
            for r in csv.DictReader(f): self.source_to_key[r['source_file']]=r['key']

    def prototypes(self):
        return [{'key':k,'point_count':int(len(v))} for k,v in sorted(self.protos.items())]

    def bootstrap_models(self, frame_rows: list[dict], id_map_pred_to_gt: dict) -> dict:
        gt_to_pred={int(v):int(k) for k,v in id_map_pred_to_gt.items()}
        models={}
        for r in frame_rows:
            gt=int(r['global_id']); pred=gt_to_pred.get(gt)
            if pred is None: continue
            key=self.source_to_key.get(r.get('source_prototype',''),'prototype_00')
            models[pred]={'prototype':key,'diameter':_num(r.get('diameter_nominal'),1.3),'length':_num(r.get('length_nominal'),1.1)}
        return models

    def _model_points(self, model:dict, center, yaw:float, observation:str, occ_side:str, severity:float, rng) -> np.ndarray:
        key=model.get('prototype','prototype_00'); proto=self.protos.get(key,self.protos['prototype_00'])
        p=_scale_proto(proto,_num(model.get('diameter'),1.3),_num(model.get('length'),1.1))
        p=_surface_perturb_local(p,observation,rng)
        if occ_side not in ('none','',None): p=_edge_occlude_local(p,occ_side,severity,rng)
        # Small reconstruction noise is point-wise; no frame-level rigid jitter.
        if len(p): p=(p+rng.normal(0,0.010,size=p.shape)).astype(np.float32)
        return _pose_local(p,center,yaw)

    def create(self, sess, request:dict, cfg:dict):
        seed=int(request.get('seed') or (2026081800+sess.current_index+len(sess.frames)*37))
        rng=np.random.default_rng(seed)
        edits={int(e['global_id']):e for e in request.get('existing',[])}
        view_mode=str(request.get('view_mode','MULTI_VIEW')).upper()
        if view_mode not in ('MULTI_VIEW','TOP_DOWN_Z'):
            raise ValueError(f'unsupported view_mode: {view_mode}')
        scene=[]; gt=[]; actions=[]; hints=[]; warnings=[]
        next_iids=list(rng.choice(np.arange(100,999,dtype=int),size=max(40,len(sess.tracks)+len(request.get('new_coils',[]))+5),replace=False))
        iid_cursor=0

        # Existing physical objects always survive. The editor intentionally has no delete/full-hide operation.
        for t in sorted(sess.tracks,key=lambda x:(x.layer,x.slot)):
            gid=int(t.global_id); e=edits.get(gid,{})
            dx=_num(e.get('dx'));dy=_num(e.get('dy'));dz=_num(e.get('dz')); dyaw=_num(e.get('dyaw_deg'))
            center=np.asarray(t.center,dtype=float)+np.array([dx,dy,dz]); yaw=float(t.yaw_deg)+dyaw
            observation=e.get('observation','normal'); side=e.get('occlusion_side','none'); sev=_num(e.get('severity'),0.0)
            model=sess.editor_models.get(gid,{'prototype':'prototype_00','diameter':t.stable_diameter,'length':t.stable_length})
            # Stable physical dimensions come from the track/model, never from visibility clipping.
            model={'prototype':model.get('prototype','prototype_00'),'diameter':float(t.stable_diameter),'length':float(t.stable_length)}
            pts=self._model_points(model,center,yaw,observation,side,sev,rng)
            if len(pts)==0: raise ValueError(f'editor invariant violated: existing ID{gid} generated zero points')
            iid=int(next_iids[iid_cursor]);iid_cursor+=1
            visibility='OCCLUDED' if side not in ('none','',None) and sev>=0.55 else ('PARTIAL_VISIBLE' if side not in ('none','',None) else 'VISIBLE')
            scene.append({'points':pts,'global_id':gid,'instance_id':iid,'layer':t.layer,'slot':t.slot,'visibility':visibility})
            gt.append({'global_id':gid,'instance_id':iid,'layer':t.layer,'slot':t.slot,'cluster':t.cluster,'visibility':visibility,'is_new':0,
                       'center_x':float(center[0]),'center_y':float(center[1]),'center_z':float(center[2]),'yaw_deg':float(yaw),
                       'diameter_nominal':float(t.stable_diameter),'length_nominal':float(t.stable_length),'source_prototype':model['prototype'],'point_count':int(len(pts))})
            actions.append({'type':'MOVE_EXISTING','global_id':gid,'dx':dx,'dy':dy,'dz':dz,'dyaw_deg':dyaw,'observation':observation,'occlusion_side':side,'severity':sev,'visible_points':int(len(pts))})

        # Build intended semantic slots for births incrementally, without exposing them to the matcher.
        virtual_tracks=[TrackState(t.global_id,t.layer,t.slot,t.cluster,t.stable_diameter,t.stable_length,np.array(t.center),t.yaw_deg,t.visibility,t.last_instance_id,t.confidence) for t in sess.tracks]
        new_specs=list(request.get('new_coils',[]))
        # Spatial order makes multi-birth slot assignment deterministic.
        new_specs=sorted(enumerate(new_specs),key=lambda kv:(_num(kv[1].get('layer'),1),_num(kv[1].get('x'))))
        for src_idx,spec in new_specs:
            D=float(np.clip(_num(spec.get('diameter'),1.3),0.6,2.2)); L=float(np.clip(_num(spec.get('length'),1.1),0.3,2.5)); layer=int(_num(spec.get('layer'),1))
            center=np.array([_num(spec.get('x')),_num(spec.get('y')),_num(spec.get('z'),0.68 if layer==1 else 1.75)],dtype=float); yaw=_num(spec.get('yaw_deg'))
            key=spec.get('prototype','prototype_00'); side=spec.get('occlusion_side','none'); sev=_num(spec.get('severity'),0.0); observation=spec.get('observation','normal')
            fake=Observation(-1000-src_idx,center,5000,D,L,yaw,1.0,layer,center-np.array([D/2,L/2,D/2]),center+np.array([D/2,L/2,D/2]),D,L)
            slot,cluster=infer_new_slot(fake,virtual_tracks,cfg)
            if slot is None:
                # Abnormal editor scenarios are allowed; choose a display slot but issue a hard warning.
                if layer==1:
                    occupied={t.slot for t in virtual_tracks if t.layer==1}; empties=[s for s in range(int(cfg['business']['layer1_max_slots'])) if s not in occupied]
                    slot=empties[0] if empties else int(cfg['business']['layer1_max_slots'])
                    cluster='abnormal'
                else:
                    slots=[t.slot for t in virtual_tracks if t.layer==2];slot=max(slots,default=-1)+1;cluster='layer2'
                warnings.append(f'NEW#{src_idx+1}: no legal semantic birth slot; generated as abnormal scenario at slot {slot}')
            used={int(t.global_id) for t in virtual_tracks}
            preferred=int(slot) if layer==1 else 10+int(slot); gid=preferred if preferred not in used else max(used|{-1})+1
            model={'prototype':key if key in self.protos else 'prototype_00','diameter':D,'length':L}
            pts=self._model_points(model,center,yaw,observation,side,sev,rng)
            if len(pts)==0: raise ValueError('NEW coil must retain visible points')
            iid=int(next_iids[iid_cursor]);iid_cursor+=1
            visibility='OCCLUDED' if side not in ('none','',None) and sev>=0.55 else ('PARTIAL_VISIBLE' if side not in ('none','',None) else 'VISIBLE')
            scene.append({'points':pts,'global_id':gid,'instance_id':iid,'layer':layer,'slot':slot,'visibility':visibility})
            gt.append({'global_id':gid,'instance_id':iid,'layer':layer,'slot':slot,'cluster':cluster,'visibility':visibility,'is_new':1,
                       'center_x':float(center[0]),'center_y':float(center[1]),'center_z':float(center[2]),'yaw_deg':float(yaw),'diameter_nominal':D,'length_nominal':L,
                       'source_prototype':model['prototype'],'point_count':int(len(pts))})
            # Robot hint can be imperfect; offsets are user-editable and are not GT identity.
            hoff=np.array([_num(spec.get('hint_dx')),_num(spec.get('hint_dy')),_num(spec.get('hint_dz'))])
            hints.append(NewCoilHint(f'user_hint_{src_idx}',D,L,center+hoff,layer))
            virtual_tracks.append(TrackState(gid,layer,int(slot),cluster,D,L,center,yaw,'VISIBLE',iid,1.0))
            sess.editor_models[gid]=model
            actions.append({'type':'ADD_NEW','intended_global_id':gid,'layer':layer,'slot':slot,'cluster':cluster,'center':center.tolist(),'yaw_deg':yaw,
                            'diameter':D,'length':L,'prototype':model['prototype'],'observation':observation,'occlusion_side':side,'severity':sev,
                            'robot_hint_center':(center+hoff).tolist(),'visible_points':int(len(pts))})

        # Apply acquisition visibility after every physical object is generated so the
        # top-camera z-buffer is global: upper-layer points can correctly hide lower
        # points in the same XY sight line. This is not a per-object z>center crop.
        if view_mode=='TOP_DOWN_Z':
            scene,view_warnings=apply_topdown_z_to_scene(scene,cfg,ensure_positive=True)
            warnings.extend(view_warnings)
            counts={int(o['global_id']):int(len(o['points'])) for o in scene}
            for g in gt:
                g['point_count']=counts.get(int(g['global_id']),0)
                g['view_mode']='TOP_DOWN_Z'
                # View-partial is distinct from physical occlusion. Preserve explicit
                # user-selected side occlusion when present.
                if g.get('visibility')=='VISIBLE': g['visibility']='VIEW_PARTIAL'
            for a in actions:
                gid=a.get('global_id',a.get('intended_global_id'))
                if gid is not None: a['visible_points']=counts.get(int(gid),a.get('visible_points',0))
                a['view_mode']='TOP_DOWN_Z'
        else:
            for g in gt: g['view_mode']='MULTI_VIEW'
            for a in actions: a['view_mode']='MULTI_VIEW'

        # Semantic-order warnings are deliberately non-blocking so the editor can create adversarial/impossible inputs.
        for layer in (1,2):
            rr=sorted([g for g in gt if int(g['layer'])==layer],key=lambda x:int(x['slot']))
            xs=[float(g['center_x']) for g in rr]
            if any(b<=a for a,b in zip(xs,xs[1:])):
                warnings.append(f'Layer {layer}: current edit reverses/collapses semantic X order; allowed as adversarial input')
        return {'seed':seed,'scene':scene,'gt':gt,'actions':actions,'hints':hints,'warnings':warnings,'view_mode':view_mode}
