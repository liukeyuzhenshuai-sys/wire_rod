from __future__ import annotations

from pathlib import Path
import csv, json, math, sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tracker.editor import write_scene_ply, _scale_proto, _surface_perturb_local, _edge_occlude_local, _pose_local
from tracker.view_model import apply_topdown_z_to_scene
from tracker.engine import load_config

CFG=load_config(ROOT/'tracker_config.json')
LABEL=ROOT/'labels'
PROTO_NPZ=np.load(ROOT/'assets'/'prototype_points.npz')
PROTOS={k:PROTO_NPZ[k].astype(np.float32) for k in PROTO_NPZ.files}
KEY_TO_SOURCE={}
with open(ROOT/'assets'/'prototype_map.csv','r',encoding='utf-8-sig',newline='') as f:
    for r in csv.DictReader(f): KEY_TO_SOURCE[r['key']]=r['source_file']

L1_D=1.22
L1_L=1.34
L1_R=L1_D/2
L1_X0=1.00
L1_SPACING=1.22
L1_Z=0.62
UP_D=1.12
UP_L=1.24
UP_R=UP_D/2
UP_Z=L1_Z+math.sqrt((L1_R+UP_R)**2-(L1_SPACING/2)**2)

# D01-D10 deliberately exercise the business semantics, not random geometry.
SEQUENCES={
 'D01':{'purpose':'Full L1 -> start L2 from left. L1 freezes; upper identity is support pair.', 'frames':[
   {'name':'full_l1_before_l2','uppers':{},'view':'MULTI_VIEW'},
   {'name':'l2_new_support0','uppers':{0:{}},'view':'MULTI_VIEW'},
   {'name':'l2_new_support1','uppers':{0:{'yaw':1.5},1:{'yaw':-2.0}},'view':'MULTI_VIEW'},
   {'name':'upper_settle_l1_locked','uppers':{0:{'dx':0.05,'yaw':2.0},1:{'dx':-0.04,'yaw':-1.0}},'view':'MULTI_VIEW'},
 ]},
 'D02':{'purpose':'Full L1 -> L2 grows from right; verifies upper slot is support valley, not append index.', 'frames':[
   {'name':'full_l1_before_right_l2','uppers':{},'view':'MULTI_VIEW'},
   {'name':'right_new_support8','uppers':{8:{'yaw':-3.0}},'view':'MULTI_VIEW'},
   {'name':'right_new_support7','uppers':{8:{'dx':-0.03},7:{'yaw':4.0}},'view':'MULTI_VIEW'},
   {'name':'right_new_support6','uppers':{8:{},7:{'dx':0.04},6:{'yaw':-4.0}},'view':'MULTI_VIEW'},
 ]},
 'D03':{'purpose':'Alternating non-contiguous upper support slots with multi-NEW. Discrete support topology dominates.', 'frames':[
   {'name':'full_l1','uppers':{},'view':'MULTI_VIEW'},
   {'name':'new_support1','uppers':{1:{'yaw':2.0}},'view':'MULTI_VIEW'},
   {'name':'new_support7','uppers':{1:{'dx':0.06},7:{'yaw':-4.0}},'view':'MULTI_VIEW'},
   {'name':'multi_new_support3_5','uppers':{1:{},3:{'yaw':5.0},5:{'yaw':-5.0},7:{'dx':-0.05}},'view':'MULTI_VIEW'},
 ]},
 'D04':{'purpose':'L2 loading under pure TOP_DOWN_Z. Upper coils physically clip lower top surfaces while every physical coil remains positively observed.', 'frames':[
   {'name':'full_l1_multiview','uppers':{},'view':'MULTI_VIEW'},
   {'name':'top_new_support2','uppers':{2:{'yaw':3.0}},'view':'TOP_DOWN_Z'},
   {'name':'top_new_support3','uppers':{2:{'dx':0.03},3:{'yaw':-3.5}},'view':'TOP_DOWN_Z'},
   {'name':'top_upper_motion','uppers':{2:{'dx':0.08,'yaw':4.0},3:{'dx':-0.07,'yaw':-4.0}},'view':'TOP_DOWN_Z'},
 ]},
 'D05':{'purpose':'Initial frame is already double-layer (multi-view). Bootstrap must infer 10-slot foundation and support-valley upper IDs without history.', 'frames':[
   {'name':'bootstrap_double_layer','uppers':{1:{'yaw':2},3:{'yaw':-3},5:{'yaw':4}},'view':'MULTI_VIEW'},
   {'name':'bootstrap_then_new_support7','uppers':{1:{'dx':0.03},3:{'dx':-0.04},5:{},7:{'yaw':-4}},'view':'MULTI_VIEW'},
   {'name':'bootstrap_followup','uppers':{1:{'dx':0.06},3:{'dx':-0.02},5:{'yaw':2},7:{'dx':0.04}},'view':'MULTI_VIEW'},
 ]},
 'D06':{'purpose':'Hard bootstrap: initial frame already double-layer and TOP_DOWN_Z with seven upper coils; many foundation coils retain only narrow positive top strips.', 'frames':[
   {'name':'bootstrap_topdown_heavy','uppers':{0:{},1:{},2:{},3:{},4:{},5:{},6:{}},'view':'TOP_DOWN_Z'},
   {'name':'topdown_add_eighth','uppers':{0:{},1:{'dx':0.02},2:{},3:{'yaw':2},4:{},5:{'dx':-0.03},6:{},7:{'yaw':-3}},'view':'TOP_DOWN_Z'},
   {'name':'topdown_all_eight_settle','uppers':{0:{'dx':0.02},1:{},2:{'dx':-0.02},3:{},4:{'dx':0.03},5:{},6:{'yaw':2},7:{'dx':-0.03}},'view':'TOP_DOWN_Z'},
 ]},
 'D07':{'purpose':'Support topology identity stress: upper coils move/yaw within their valleys; L1 remains a locked reference lattice.', 'frames':[
   {'name':'support_identity_start','uppers':{0:{},2:{},4:{},6:{}},'view':'MULTI_VIEW'},
   {'name':'support_offsets','uppers':{0:{'dx':0.34,'yaw':6},2:{'dx':-0.31,'yaw':-6},4:{'dx':0.29},6:{'dx':-0.33}},'view':'MULTI_VIEW'},
   {'name':'support_offsets_rebound','uppers':{0:{'dx':0.10},2:{'dx':-0.12},4:{'dx':0.08,'yaw':3},6:{'dx':-0.09,'yaw':-3}},'view':'MULTI_VIEW'},
 ]},
 'D08':{'purpose':'Collapse-risk diagnostic: once L2 is active, a first-layer foundation coil shifts beyond the hard lock. Must surface ABNORMAL, never silently relearn foundation.', 'frames':[
   {'name':'locked_foundation_start','uppers':{3:{},5:{}},'view':'MULTI_VIEW'},
   {'name':'foundation_hard_shift','uppers':{3:{},5:{}},'view':'MULTI_VIEW','l1_shift':{4:{'dx':0.20}}},
   {'name':'foundation_returns','uppers':{3:{'dx':0.02},5:{'dx':-0.02}},'view':'MULTI_VIEW'},
 ]},
 'D09':{'purpose':'Upper off-support abnormality: historical upper coil leaves its support valley. Must report support topology abnormality instead of hopping ID to adjacent valley.', 'frames':[
   {'name':'upper_support_start','uppers':{4:{}},'view':'MULTI_VIEW'},
   {'name':'upper_off_support','uppers':{4:{'dx':0.78,'yaw':5}},'view':'MULTI_VIEW'},
   {'name':'upper_back_on_support','uppers':{4:{'dx':0.04,'yaw':1}},'view':'MULTI_VIEW'},
 ]},
 'D10':{'purpose':'L2 capacity lifecycle: grow to realistic maximum 8, then intentionally request a ninth upper coil to verify capacity abnormality.', 'frames':[
   {'name':'full_l1','uppers':{},'view':'MULTI_VIEW'},
   {'name':'multi_new_0_7','uppers':{0:{},7:{}},'view':'MULTI_VIEW'},
   {'name':'multi_new_1_6','uppers':{0:{},1:{'yaw':2},6:{'yaw':-2},7:{}},'view':'MULTI_VIEW'},
   {'name':'multi_new_2_5','uppers':{0:{},1:{},2:{'yaw':3},5:{'yaw':-3},6:{},7:{}},'view':'MULTI_VIEW'},
   {'name':'multi_new_3_4_reach8','uppers':{0:{},1:{},2:{},3:{'yaw':4},4:{'yaw':-4},5:{},6:{},7:{}},'view':'MULTI_VIEW'},
   {'name':'illegal_ninth_support8','uppers':{0:{},1:{},2:{},3:{},4:{},5:{},6:{},7:{},8:{'yaw':5}},'view':'MULTI_VIEW'},
 ]},
}


def read_csv(name):
    p=LABEL/name
    if not p.exists(): return []
    with p.open('r',encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))

def write_csv(name,rows,fields=None):
    p=LABEL/name
    if fields is None: fields=list(rows[0].keys()) if rows else []
    with p.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)

def l1_center(slot, shift=None):
    sh=shift or {}
    return np.array([L1_X0+slot*L1_SPACING+float(sh.get('dx',0)),float(sh.get('dy',0)),L1_Z+float(sh.get('dz',0))],dtype=float)

def support_center(slot, l1_shifts):
    a=l1_center(slot,l1_shifts.get(slot));b=l1_center(slot+1,l1_shifts.get(slot+1))
    return np.array([(a[0]+b[0])/2,(a[1]+b[1])/2,UP_Z],dtype=float)

def visibility_event(a,b): return 'VISIBILITY_STABLE' if a==b else f'{a}_TO_{b}'

def make_points(gid, D, L, center, yaw, observation, occ_side, severity, rng):
    key=f'prototype_{gid%7:02d}'
    proto=PROTOS[key]
    p=_scale_proto(proto,D,L)
    p=_surface_perturb_local(p,observation,rng)
    if occ_side not in ('none','',None):p=_edge_occlude_local(p,occ_side,severity,rng)
    if len(p):p=(p+rng.normal(0,0.010,size=p.shape)).astype(np.float32)
    return _pose_local(p,center,yaw),key

def gen_frame(seq, fi, spec, prev_upper_slots):
    rng=np.random.default_rng(202608180800 + sum(ord(c) for c in seq)*31 + fi*1009)
    l1_shifts={int(k):v for k,v in spec.get('l1_shift',{}).items()}
    objects=[]; meta=[]; pre_counts={}
    iids=list(rng.choice(np.arange(100,999,dtype=int),size=40,replace=False));ic=0
    # full L1 is always physically present in every D frame.
    for slot in range(10):
        gid=slot;center=l1_center(slot,l1_shifts.get(slot));yaw=float(l1_shifts.get(slot,{}).get('yaw',0.0))
        obs='roll_mild' if (fi+slot)%9==0 else 'normal';side='none';sev=0.0
        pts,key=make_points(gid,L1_D,L1_L,center,yaw,obs,side,sev,rng);iid=int(iids[ic]);ic+=1
        objects.append({'points':pts,'global_id':gid,'instance_id':iid,'layer':1,'slot':slot,'visibility':'VISIBLE'})
        pre_counts[gid]=len(pts)
        meta.append({'gid':gid,'iid':iid,'layer':1,'slot':slot,'cluster':'full_L1','center':center,'yaw':yaw,'D':L1_D,'L':L1_L,'key':key,'obs':obs,'side':side,'severity':sev,'is_new':0})
    uppers={int(k):v for k,v in spec.get('uppers',{}).items()}
    for slot in sorted(uppers):
        a=uppers[slot] or {};gid=10+slot;base=support_center(slot,l1_shifts)
        center=base+np.array([float(a.get('dx',0)),float(a.get('dy',0)),float(a.get('dz',0))])
        yaw=float(a.get('yaw',0));obs=a.get('observation','normal');side=a.get('side','none');sev=float(a.get('severity',0))
        pts,key=make_points(gid,UP_D,UP_L,center,yaw,obs,side,sev,rng);iid=int(iids[ic]);ic+=1
        objects.append({'points':pts,'global_id':gid,'instance_id':iid,'layer':2,'slot':slot,'visibility':'VISIBLE'})
        pre_counts[gid]=len(pts)
        meta.append({'gid':gid,'iid':iid,'layer':2,'slot':slot,'cluster':'layer2','center':center,'yaw':yaw,'D':UP_D,'L':UP_L,'key':key,'obs':obs,'side':side,'severity':sev,'is_new':int(slot not in prev_upper_slots) if fi>0 else 0})
    warnings=[]
    if spec.get('view')=='TOP_DOWN_Z':
        objects,warnings=apply_topdown_z_to_scene(objects,CFG,ensure_positive=True)
    counts={int(o['global_id']):len(o['points']) for o in objects}
    by_gid={m['gid']:m for m in meta}
    for o in objects:
        m=by_gid[int(o['global_id'])]
        if spec.get('view')=='TOP_DOWN_Z':
            ratio=counts[m['gid']]/max(pre_counts[m['gid']],1)
            # Lower coils with strong upper projection are true positive-partial occlusions;
            # otherwise TOP_DOWN_Z is only a view-limited partial observation.
            if m['layer']==1 and ratio<0.42:
                o['visibility']='OCCLUDED';m['visibility']='OCCLUDED';m['obs']='top_down_z+upper_occlusion'
            else:
                o['visibility']='VIEW_PARTIAL';m['visibility']='VIEW_PARTIAL';m['obs']='top_down_z'
        else:
            o['visibility']='VISIBLE';m['visibility']='VISIBLE'
        m['count']=counts[m['gid']];m['pre_count']=pre_counts[m['gid']]
    return objects,meta,warnings,l1_shifts

def main():
    dseq=set(SEQUENCES)
    table_names=['frame_manifest.csv','object_gt.csv','sequence_index.csv','frame_pairs.csv','transition_gt.csv','frame_semantics.csv','neighbor_gt.csv','complex_sequence_catalog.csv']
    tables={n:read_csv(n) for n in table_names}
    fields={n:(list(tables[n][0].keys()) if tables[n] else []) for n in table_names}
    # idempotent cleanup
    for n in table_names:
        tables[n]=[r for r in tables[n] if r.get('sequence_id') not in dseq]
    for p in ROOT.glob('D*_F*.ply'):p.unlink()
    max_pair=max([int(r['pair_id'].split('_')[-1]) for r in tables['frame_pairs.csv'] if r.get('pair_id','').startswith('pair_')]+[-1])
    support_rows=[];generation=[];fg_new=[]
    sup_fields=['file','sequence_id','frame_index','upper_global_id','upper_slot','lower_left_global_id','lower_right_global_id','support_center_x','support_center_y','support_center_z','upper_center_x','upper_center_y','upper_center_z','support_offset_xy_m','relation']
    for seq,sdef in SEQUENCES.items():
        prev_meta=None;prev_file=None;prev_upper=set();files=[]
        for fi,spec in enumerate(sdef['frames']):
            objects,meta,warnings,l1_shifts=gen_frame(seq,fi,spec,prev_upper)
            file=f'{seq}_F{fi:02d}_{spec["name"]}.ply';npts=write_scene_ply(ROOT/file,objects);files.append(file)
            scene_name=spec['name'];view=spec.get('view','MULTI_VIEW'); by_gid={m['gid']:m for m in meta}
            # Robot hint only for births after F0; small deterministic target error.
            for m in meta:
                target=['','','']
                if m['is_new']:
                    target=[m['center'][0]+0.06*(-1 if m['slot']%2 else 1),m['center'][1]+0.03,m['center'][2]]
                prev=prev_meta.get(m['gid']) if prev_meta else None
                dx='' if prev is None else float(m['center'][0]-prev['center'][0]);dy='' if prev is None else float(m['center'][1]-prev['center'][1]);dz='' if prev is None else float(m['center'][2]-prev['center'][2])
                note=('full first-layer foundation; movement after L2 activation is collapse-risk abnormality' if m['layer']==1 else f'upper slot {m["slot"]} supported by L1[{m["slot"]},{m["slot"]+1}]')
                tables['object_gt.csv'].append({'file':file,'sequence_id':seq,'frame_index':str(fi),'scene_name':scene_name,'global_id':str(m['gid']),'instance_id':str(m['iid']),'layer':str(m['layer']),'slot':str(m['slot']),'cluster':m['cluster'],'visibility':m['visibility'],'is_new':str(m['is_new']),'source_prototype':KEY_TO_SOURCE[m['key']],'point_count':str(m['count']),'center_x':str(float(m['center'][0])),'center_y':str(float(m['center'][1])),'center_z':str(float(m['center'][2])),'yaw_deg':str(float(m['yaw'])),'diameter_nominal':str(m['D']),'length_nominal':str(m['L']),'dx_from_prev':str(dx),'dy_from_prev':str(dy),'dz_from_prev':str(dz),'observation':m['obs'],'severity':str(m['severity']),'occlusion_side':m['side'],'target_x':str(target[0]),'target_y':str(target[1]),'target_z':str(target[2]),'note':note})
            num_occ=sum(m['visibility']=='OCCLUDED' for m in meta);num_partial=sum(m['visibility'] in ('PARTIAL_VISIBLE','VIEW_PARTIAL') for m in meta)
            upslots=sorted(m['slot'] for m in meta if m['layer']==2)
            tags=f'double_layer;full_l1;support_topology;{view.lower()}'
            if seq in ('D08','D09') or (seq=='D10' and fi==len(sdef['frames'])-1): tags+=';abnormal'
            tables['frame_manifest.csv'].append({'file':file,'sequence_id':seq,'frame_index':str(fi),'scene_name':scene_name,'num_objects_total':str(len(meta)),'num_visible_objects':str(len(meta)),'num_points':str(npts),'tags':tags,'description':sdef['purpose']})
            tables['frame_semantics.csv'].append({'file':file,'sequence_id':seq,'frame_index':str(fi),'layer1_occupied_slots':';'.join(map(str,range(10))),'layer1_unoccupied_slots':'','left_cluster_slots':'','right_cluster_slots':'','full_L1_slots':';'.join(map(str,range(10))),'normal_middle_free_slots':'','layer2_slots':';'.join(map(str,upslots)),'num_new':str(sum(m['is_new'] for m in meta)),'num_partial':str(num_partial),'num_occluded':str(num_occ)})
            tables['complex_sequence_catalog.csv'].append({'sequence_id':seq,'frame_index':str(fi),'file':file,'difficulty':'5' if seq in ('D06','D08','D09','D10') else '4','tags':tags,'description':sdef['purpose']})
            # L1 neighbor GT
            for k in range(9):
                a=by_gid[k];b=by_gid[k+1];dx=float(b['center'][0]-a['center'][0]);dy=float(b['center'][1]-a['center'][1]);gap=abs(dx)-(a['D']+b['D'])/2
                tables['neighbor_gt.csv'].append({'file':file,'sequence_id':seq,'frame_index':str(fi),'layer':'1','cluster':'full_L1','left_global_id':str(k),'right_global_id':str(k+1),'left_slot':str(k),'right_slot':str(k+1),'slot_delta':'1','center_dx_m':str(dx),'center_dy_m':str(dy),'center_xy_distance_m':str(math.hypot(dx,dy)),'model_surface_gap_x_m':str(gap),'relation':'DIRECT_CLUSTER_NEIGHBOR'})
            # Upper support GT and upper-neighbor rows when support slots are contiguous.
            for m in [x for x in meta if x['layer']==2]:
                sc=support_center(m['slot'],l1_shifts);err=float(np.linalg.norm(m['center'][:2]-sc[:2]));rel='SUPPORTED_BY' if err<=float(CFG['double_layer']['support_slot_hard_center_error_m']) else 'ABNORMAL_OFF_SUPPORT'
                support_rows.append({'file':file,'sequence_id':seq,'frame_index':str(fi),'upper_global_id':str(m['gid']),'upper_slot':str(m['slot']),'lower_left_global_id':str(m['slot']),'lower_right_global_id':str(m['slot']+1),'support_center_x':str(float(sc[0])),'support_center_y':str(float(sc[1])),'support_center_z':str(float(sc[2])),'upper_center_x':str(float(m['center'][0])),'upper_center_y':str(float(m['center'][1])),'upper_center_z':str(float(m['center'][2])),'support_offset_xy_m':str(err),'relation':rel})
            ups=sorted([m for m in meta if m['layer']==2],key=lambda x:x['slot'])
            for a,b in zip(ups,ups[1:]):
                if b['slot']-a['slot']!=1:continue
                dx=float(b['center'][0]-a['center'][0]);dy=float(b['center'][1]-a['center'][1]);gap=abs(dx)-(a['D']+b['D'])/2
                tables['neighbor_gt.csv'].append({'file':file,'sequence_id':seq,'frame_index':str(fi),'layer':'2','cluster':'layer2','left_global_id':str(a['gid']),'right_global_id':str(b['gid']),'left_slot':str(a['slot']),'right_slot':str(b['slot']),'slot_delta':'1','center_dx_m':str(dx),'center_dy_m':str(dy),'center_xy_distance_m':str(math.hypot(dx,dy)),'model_surface_gap_x_m':str(gap),'relation':'DIRECT_CLUSTER_NEIGHBOR'})
            fg_new.append({'file':file,'sequence_id':seq,'frame_index':fi,'scene_name':scene_name,'num_points':npts,'view_mode':view,'second_layer_signal':bool(upslots),'objects':[{'global_id':m['gid'],'instance_id':m['iid'],'layer':m['layer'],'slot':m['slot'],'cluster':m['cluster'],'visibility':m['visibility'],'is_new':bool(m['is_new']),'center_gt':[float(x) for x in m['center']],'yaw_gt_deg':m['yaw'],'diameter_nominal_m':m['D'],'length_nominal_m':m['L'],'observation':m['obs'],'point_count':m['count'],'supported_by':([m['slot'],m['slot']+1] if m['layer']==2 else None)} for m in meta]})
            generation.append({'file':file,'sequence_id':seq,'frame_index':fi,'view_mode':view,'objects':len(meta),'points':npts,'warnings':warnings,'min_object_points':min(m['count'] for m in meta),'lower_point_counts':{str(m['gid']):m['count'] for m in meta if m['layer']==1}})
            if prev_file is not None:
                max_pair+=1;pid=f'pair_{max_pair:03d}'
                tables['frame_pairs.csv'].append({'pair_id':pid,'sequence_id':seq,'pair_type':'temporal_adjacent','previous_file':prev_file,'current_file':file,'previous_frame_index':str(fi-1),'current_frame_index':str(fi),'recommended_for_id_eval':'1'})
                cur_by={m['gid']:m for m in meta};prev_by=prev_meta
                for gid,m in cur_by.items():
                    old=prev_by.get(gid);new=old is None
                    if new: dx=dy=dz=disp='';pi='';pv='';rel='NEW';ve='NEW'
                    else:
                        dx=float(m['center'][0]-old['center'][0]);dy=float(m['center'][1]-old['center'][1]);dz=float(m['center'][2]-old['center'][2]);disp=math.sqrt(dx*dx+dy*dy+dz*dz);pi=str(old['iid']);pv=old['visibility'];rel='SAME_OBJECT';ve=visibility_event(pv,m['visibility'])
                    tables['transition_gt.csv'].append({'pair_id':pid,'sequence_id':seq,'pair_type':'temporal_adjacent','previous_file':prev_file,'current_file':file,'global_id':str(gid),'previous_instance_id':pi,'current_instance_id':str(m['iid']),'relation_gt':rel,'visibility_event_gt':ve,'previous_visibility':pv,'current_visibility':m['visibility'],'is_new_gt':'1' if new else '0','dx_gt_m':str(dx),'dy_gt_m':str(dy),'dz_gt_m':str(dz),'displacement_gt_m':str(disp),'layer_gt':str(m['layer']),'slot_gt':str(m['slot'])})
            prev_meta={m['gid']:m for m in meta};prev_file=file;prev_upper={int(k) for k in spec.get('uppers',{})}
        tables['sequence_index.csv'].append({'sequence_id':seq,'relation_mode':'temporal_adjacent','num_frames':str(len(files)),'first_file':files[0],'last_file':files[-1],'purpose':sdef['purpose']})
    for n in table_names: write_csv(n,tables[n],fields[n])
    write_csv('support_gt.csv',support_rows,sup_fields)
    # frame_gt jsonl: replace old D lines.
    fg=LABEL/'frame_gt.jsonl';old=[]
    if fg.exists():
        for line in fg.read_text(encoding='utf-8').splitlines():
            if line.strip():
                x=json.loads(line)
                if x.get('sequence_id') not in dseq:old.append(x)
    with fg.open('w',encoding='utf-8') as f:
        for x in old+fg_new:f.write(json.dumps(x,ensure_ascii=False,separators=(',',':'))+'\n')
    (ROOT/'results'/'double_layer_dataset_generation.json').write_text(json.dumps({'version':'v0.8','sequences':{k:v['purpose'] for k,v in SEQUENCES.items()},'frames_added':len(generation),'frames':generation},ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'generated D01-D10: frames={len(generation)}, total_root_ply={len(list(ROOT.glob("*.ply")))}')

if __name__=='__main__': main()
