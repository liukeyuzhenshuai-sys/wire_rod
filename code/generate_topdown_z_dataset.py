from __future__ import annotations

from pathlib import Path
import csv, json, math, sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tracker.ply import read_ply
from tracker.editor import write_scene_ply, _scale_proto, _edge_occlude_local, _pose_local
from tracker.view_model import apply_topdown_z_to_scene
from tracker.engine import load_config

CFG=load_config(ROOT/'tracker_config.json')
LABEL=ROOT/'labels'

DERIVED={
    'Z01':{'source':'S05','mode':'TOP_DOWN_Z','purpose':'Pure -Z top-camera sequence derived from S05: yaw + nonuniform historical motion; only top-visible surface is retained.'},
    'Z02':{'source':'C03','mode':'TOP_DOWN_Z','purpose':'Pure -Z top-camera two-edge-cluster sequence with NEW, gaps, motion and positive partial occlusion.'},
    'Z03':{'source':'C06','mode':'TOP_DOWN_Z','purpose':'Pure -Z top-camera full L1 to L2 loading sequence; global z-buffer makes upper coils physically occlude lower projected surfaces.'},
    'Z04':{'source':'C04','mode':'TOP_DOWN_Z','purpose':'Pure -Z top-camera adversarial same-geometry / nearest-neighbor trap sequence; identity must come from semantics, not full-cylinder geometry.'},
    'Z05':{'source':'S05','mode':'MIXED','purpose':'View-switch sequence: multi-view -> top-only -Z -> top-only -Z -> multi-view, testing persistent identity across acquisition-mode changes.'},
}


def read_csv(name):
    with open(LABEL/name,'r',encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))

def write_csv(name,rows,fieldnames=None):
    if fieldnames is None:
        fieldnames=list(rows[0].keys()) if rows else []
    with open(LABEL/name,'w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fieldnames,extrasaction='ignore');w.writeheader();w.writerows(rows)

def scene_from_array(arr,gt_rows):
    gt_by_iid={int(float(r['instance_id'])):r for r in gt_rows if r.get('instance_id','') not in ('',None)}
    scene=[]
    for iid in np.unique(arr['instance_id']):
        iid=int(iid); m=arr['instance_id']==iid; r=gt_by_iid[iid]
        pts=np.column_stack([arr['x'][m],arr['y'][m],arr['z'][m]]).astype(np.float32)
        scene.append({'points':pts,'global_id':int(r['global_id']),'instance_id':iid,'layer':int(r['layer']),'slot':int(r['slot']),'visibility':r['visibility']})
    return scene


# V0.7 top-down sets follow the current business rule: every historical physical
# coil remains positively observable. Legacy V3/V4 source sequences contain a few
# zero-point OCCLUDED rows and one duplicated synthetic instance; normalize those
# artifacts while deriving Z-view data rather than propagating obsolete semantics.
_PROTO_NPZ=np.load(ROOT/'assets'/'prototype_points.npz')
_PROTOS={k:_PROTO_NPZ[k].astype(np.float32) for k in _PROTO_NPZ.files}
_SOURCE_TO_KEY={}
with open(ROOT/'assets'/'prototype_map.csv','r',encoding='utf-8-sig',newline='') as _f:
    for _r in csv.DictReader(_f): _SOURCE_TO_KEY[_r['source_file']]=_r['key']

def _unique_gt_by_gid(gt_rows):
    by={}
    def rank(r):
        # For a duplicated birth artifact, the row carrying is_new/robot target is the
        # authoritative semantic row; otherwise prefer a positive instance.
        return (int(r.get('is_new') or 0), int(float(r.get('instance_id',-1)))>=0)
    for r in gt_rows:
        gid=int(r['global_id'])
        if gid not in by or rank(r)>rank(by[gid]): by[gid]=dict(r)
    return by

def normalized_scene_from_array(arr,gt_rows,frame_index):
    """One physical object per global ID + positive partial restoration.

    This function is dataset generation only. global_id is GT carried by the synthetic
    PLY and is never exposed to the matcher.
    """
    gt=_unique_gt_by_gid(gt_rows)
    scene=[]; normalized=[]; notes=[]
    used_iids={int(x) for x in np.unique(arr['instance_id'])}
    for gid in sorted(gt):
        r=dict(gt[gid]); chosen_iid=None; pts=None
        if 'global_id' in arr.dtype.names:
            gm=arr['global_id']==gid
            iids=[int(x) for x in np.unique(arr['instance_id'][gm])] if np.any(gm) else []
            if iids:
                # Legacy duplicate-instance artifact: retain one physical observation.
                preferred=int(float(r.get('instance_id',-1)))
                chosen_iid=preferred if preferred in iids else max(iids,key=lambda iid:int(np.sum(gm & (arr['instance_id']==iid))))
                m=gm & (arr['instance_id']==chosen_iid)
                pts=np.column_stack([arr['x'][m],arr['y'][m],arr['z'][m]]).astype(np.float32)
                if len(iids)>1:
                    notes.append(f'gid{gid}: deduplicated legacy instance_ids={iids} -> {chosen_iid}')
        if pts is None or len(pts)==0:
            # Legacy zero-point OCCLUDED is converted to a heavy but positive partial
            # observation from the same real prototype, at the GT physical pose.
            key=_SOURCE_TO_KEY.get(r.get('source_prototype',''),'prototype_00')
            proto=_PROTOS.get(key,_PROTOS['prototype_00'])
            D=float(r['diameter_nominal']);L=float(r['length_nominal']);yaw=float(r['yaw_deg'])
            center=np.array([float(r['center_x']),float(r['center_y']),float(r['center_z'])],dtype=float)
            local=_scale_proto(proto,D,L)
            rng=np.random.default_rng(2026081807 + int(frame_index)*1009 + gid*37)
            side=r.get('occlusion_side') or 'both';sev=max(float(r.get('severity') or 0.85),0.75)
            local=_edge_occlude_local(local,side,sev,rng)
            # Small reconstruction perturbation, then place at the source frame pose.
            if len(local): local=(local+rng.normal(0,0.010,size=local.shape)).astype(np.float32)
            pts=_pose_local(local,center,yaw)
            chosen_iid=2000+int(frame_index)*100+gid
            while chosen_iid in used_iids: chosen_iid+=1
            used_iids.add(chosen_iid)
            r['instance_id']=str(chosen_iid); r['visibility']='OCCLUDED'; r['point_count']=str(len(pts))
            notes.append(f'gid{gid}: restored obsolete zero-point occlusion as positive partial instance {chosen_iid} ({len(pts)} pts)')
        else:
            r['instance_id']=str(chosen_iid);r['point_count']=str(len(pts))
        scene.append({'points':pts,'global_id':gid,'instance_id':int(chosen_iid),'layer':int(r['layer']),'slot':int(r['slot']),'visibility':r['visibility']})
        normalized.append(r)
    return scene,normalized,notes

def visibility_for_topdown(src_vis):
    if src_vis=='OCCLUDED': return 'OCCLUDED'
    if src_vis=='PARTIAL_VISIBLE': return 'PARTIAL_VISIBLE'
    return 'VIEW_PARTIAL'

def transition_event(a,b):
    if a==b:return 'VISIBILITY_STABLE'
    return f'{a}_TO_{b}'

def main():
    manifest=read_csv('frame_manifest.csv'); objects=read_csv('object_gt.csv'); seqidx=read_csv('sequence_index.csv')
    pairs=read_csv('frame_pairs.csv'); transitions=read_csv('transition_gt.csv')
    semantics=read_csv('frame_semantics.csv'); neighbors=read_csv('neighbor_gt.csv'); catalog=read_csv('complex_sequence_catalog.csv')
    manifest_fields=list(manifest[0].keys()); object_fields=list(objects[0].keys()); seq_fields=list(seqidx[0].keys()); pair_fields=list(pairs[0].keys()); trans_fields=list(transitions[0].keys())
    sem_fields=list(semantics[0].keys()); nei_fields=list(neighbors[0].keys()); cat_fields=list(catalog[0].keys())

    # Idempotent: remove prior Z-derived rows and files.
    zseq=set(DERIVED)
    manifest=[r for r in manifest if r['sequence_id'] not in zseq]
    objects=[r for r in objects if r['sequence_id'] not in zseq]
    seqidx=[r for r in seqidx if r['sequence_id'] not in zseq]
    pairs=[r for r in pairs if r['sequence_id'] not in zseq]
    transitions=[r for r in transitions if r['sequence_id'] not in zseq]
    semantics=[r for r in semantics if r['sequence_id'] not in zseq]
    neighbors=[r for r in neighbors if r['sequence_id'] not in zseq]
    catalog=[r for r in catalog if r['sequence_id'] not in zseq]
    for p in ROOT.glob('Z0*_F*.ply'): p.unlink()

    src_manifest=list(manifest); src_objects=list(objects); src_semantics=list(semantics); src_neighbors=list(neighbors); src_catalog=list(catalog)
    max_pair=max([int(r['pair_id'].split('_')[-1]) for r in pairs if r['pair_id'].startswith('pair_')]+[-1])
    frame_gt_new=[]; generation=[]

    for zid,spec in DERIVED.items():
        src=spec['source']
        frames=sorted([r for r in src_manifest if r['sequence_id']==src],key=lambda r:int(r['frame_index']))
        if not frames: raise RuntimeError(f'missing source sequence {src}')
        z_files=[]; z_obj_by_frame={}
        for j,fr in enumerate(frames):
            srcfile=fr['file']; src_scene=fr['scene_name']; top=(spec['mode']=='TOP_DOWN_Z' or (spec['mode']=='MIXED' and j in (1,2)))
            suffix=src_scene.replace(' ','_')
            dst=f'{zid}_F{j:02d}_{"topdown" if top else "multiview"}_{suffix}.ply'
            src_gt=[dict(r) for r in src_objects if r['file']==srcfile]
            arr=read_ply(ROOT/srcfile)
            scene,src_gt,normalization_notes=normalized_scene_from_array(arr,src_gt,j)
            warnings=list(normalization_notes)
            if top:
                scene,warnings=apply_topdown_z_to_scene(scene,CFG,ensure_positive=True)
                # Top-down is a view mode, not physical occlusion.
                for o in scene:
                    g=next(r for r in src_gt if int(r['global_id'])==int(o['global_id']))
                    o['visibility']=visibility_for_topdown(g['visibility'])
            n=write_scene_ply(ROOT/dst,scene)
            counts={int(o['global_id']):len(o['points']) for o in scene}
            zrows=[]
            for r in src_gt:
                q=dict(r); q['file']=dst;q['sequence_id']=zid;q['frame_index']=str(j);q['scene_name']=('topdown_z_' if top else 'multiview_')+src_scene
                q['point_count']=str(counts.get(int(q['global_id']),0))
                if top:
                    q['visibility']=visibility_for_topdown(r['visibility'])
                    oldobs=r.get('observation','clean')
                    q['observation']='top_down_z' if oldobs in ('','clean','normal') else f'top_down_z+{oldobs}'
                    q['note']=(r.get('note','')+' | global -Z XY z-buffer top-visible surface only; view partial is not occlusion').strip(' |')
                zrows.append(q)
            objects.extend(zrows); z_obj_by_frame[dst]=zrows; z_files.append(dst)

            mf=dict(fr);mf['file']=dst;mf['sequence_id']=zid;mf['frame_index']=str(j);mf['scene_name']=q['scene_name'];mf['num_points']=str(n)
            mf['num_visible_objects']=str(sum(int(r['point_count'])>0 for r in zrows));mf['tags']=(fr.get('tags','')+';'+('topdown_z;z_projection_only' if top else 'multiview;view_switch')).strip(';')
            mf['description']=f"{spec['purpose']} Source={srcfile}. {'Only -Z top-visible points.' if top else 'Multi-view transition frame.'}"
            manifest.append(mf)

            # Copy semantic topology and neighbor GT; centers/slots are physical GT and unchanged.
            for sr in src_semantics:
                if sr['file']==srcfile:
                    q=dict(sr);q['file']=dst;q['sequence_id']=zid;q['frame_index']=str(j);semantics.append(q)
            for nr in src_neighbors:
                if nr['file']==srcfile:
                    q=dict(nr);q['file']=dst;q['sequence_id']=zid;q['frame_index']=str(j);neighbors.append(q)
            catalog.append({'sequence_id':zid,'frame_index':str(j),'file':dst,'difficulty':'4' if top else '3','tags':('topdown_z;z_projection_only;'+fr.get('tags','')).strip(';'),'description':mf['description']})

            # JSONL handoff record.
            jobjs=[]
            for r in zrows:
                jobjs.append({'global_id':int(r['global_id']),'instance_id':int(float(r['instance_id'])),'layer':int(r['layer']),'slot':int(r['slot']),'cluster':r['cluster'],'visibility':r['visibility'],'is_new':bool(int(r['is_new'])),'center_gt':[float(r['center_x']),float(r['center_y']),float(r['center_z'])],'yaw_gt_deg':float(r['yaw_deg']),'diameter_nominal_m':float(r['diameter_nominal']),'length_nominal_m':float(r['length_nominal']),'observation':r['observation'],'severity':float(r['severity'] or 0),'occlusion_side':r['occlusion_side'],'source_prototype':r['source_prototype'],'robot_target':None if not r.get('target_x') else [float(r['target_x']),float(r['target_y']),float(r['target_z'])]})
            frame_gt_new.append({'file':dst,'sequence_id':zid,'frame_index':j,'scene_name':mf['scene_name'],'num_points':n,'view_mode':'TOP_DOWN_Z' if top else 'MULTI_VIEW','objects':jobjs})
            generation.append({'file':dst,'source_file':srcfile,'topdown':top,'num_points':n,'warnings':warnings,'object_counts':counts})

        seqidx.append({'sequence_id':zid,'relation_mode':'temporal_adjacent','num_frames':str(len(z_files)),'first_file':z_files[0],'last_file':z_files[-1],'purpose':spec['purpose']})
        # transitions / frame-pair index
        for j in range(1,len(z_files)):
            max_pair+=1; pid=f'pair_{max_pair:03d}'; prev=z_files[j-1];cur=z_files[j]
            pairs.append({'pair_id':pid,'sequence_id':zid,'pair_type':'temporal_adjacent','previous_file':prev,'current_file':cur,'previous_frame_index':str(j-1),'current_frame_index':str(j),'recommended_for_id_eval':'0'})
            pr={int(r['global_id']):r for r in z_obj_by_frame[prev]}; cr={int(r['global_id']):r for r in z_obj_by_frame[cur]}
            for gid,r in cr.items():
                old=pr.get(gid); isnew=1 if old is None else 0
                if old is None:
                    dx=dy=dz=disp='';pi='';pv='';rel='NEW'
                else:
                    dx=float(r['center_x'])-float(old['center_x']);dy=float(r['center_y'])-float(old['center_y']);dz=float(r['center_z'])-float(old['center_z']);disp=math.sqrt(dx*dx+dy*dy+dz*dz);pi=old['instance_id'];pv=old['visibility'];rel='SAME_OBJECT'
                transitions.append({'pair_id':pid,'sequence_id':zid,'pair_type':'temporal_adjacent','previous_file':prev,'current_file':cur,'global_id':str(gid),'previous_instance_id':pi,'current_instance_id':r['instance_id'],'relation_gt':rel,'visibility_event_gt':('NEW' if old is None else transition_event(pv,r['visibility'])),'previous_visibility':pv,'current_visibility':r['visibility'],'is_new_gt':str(isnew),'dx_gt_m':dx,'dy_gt_m':dy,'dz_gt_m':dz,'displacement_gt_m':disp,'layer_gt':r['layer'],'slot_gt':r['slot']})

    write_csv('frame_manifest.csv',manifest,manifest_fields);write_csv('object_gt.csv',objects,object_fields);write_csv('sequence_index.csv',seqidx,seq_fields)
    write_csv('frame_pairs.csv',pairs,pair_fields);write_csv('transition_gt.csv',transitions,trans_fields);write_csv('frame_semantics.csv',semantics,sem_fields);write_csv('neighbor_gt.csv',neighbors,nei_fields);write_csv('complex_sequence_catalog.csv',catalog,cat_fields)
    # Replace any previous Z-derived JSONL lines and append current ones.
    fg=LABEL/'frame_gt.jsonl'; old=[]
    if fg.exists():
        for line in fg.read_text(encoding='utf-8').splitlines():
            if not line.strip(): continue
            x=json.loads(line)
            if x.get('sequence_id') not in zseq:old.append(x)
    with fg.open('w',encoding='utf-8') as f:
        for x in old+frame_gt_new:f.write(json.dumps(x,ensure_ascii=False,separators=(',',':'))+'\n')
    (ROOT/'results'/'topdown_dataset_generation.json').write_text(json.dumps({'sequences':DERIVED,'frames_added':len(generation),'frames':generation},ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'generated {len(generation)} Z-view frames; total PLY={len(list(ROOT.glob("*.ply")))}')

if __name__=='__main__':main()
