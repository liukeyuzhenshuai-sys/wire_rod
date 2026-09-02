"""Self-contained V3.1 generator.
Requires only: numpy + this package's assets/source_real_coils.zip and assets/scene_spec.csv.
It does NOT depend on the original chat, V2 package, Open3D, pandas, or network access.
"""
from pathlib import Path
import argparse,csv,hashlib,math,re,shutil,zipfile
import numpy as np
SEED=20260818
PALETTE=np.array([
[230,70,70],[70,180,240],[70,210,130],[245,190,50],[180,95,235],[40,200,205],[245,120,45],[130,210,65],[235,90,180],[100,130,245],
[215,150,75],[75,210,195],[220,105,120],[135,95,225],[100,200,100],[235,155,210],[85,165,230],[210,210,75],[165,120,85],[90,215,160],
[245,95,85],[95,195,245],[110,225,135],[250,200,90],[195,115,245],[75,220,220],[250,135,70],[150,220,95],[245,115,195],[120,145,250],
[225,165,90],[90,220,205],[230,125,140],[155,115,235],[120,215,120],[245,175,220],[105,180,240],[220,220,100],[180,135,100],[110,225,175]],dtype=np.uint8)
TYPEMAP={'float':'<f4','float32':'<f4','double':'<f8','uchar':'u1','uint8':'u1','char':'i1','short':'<i2','ushort':'<u2','int':'<i4','uint':'<u4'}

def read_ply_xyz(path):
    with open(path,'rb') as f:
        h=[]
        while True:
            ln=f.readline();h.append(ln)
            if ln.strip()==b'end_header':break
        text=b''.join(h).decode('ascii','ignore');n=int(re.search(r'element vertex\s+(\d+)',text).group(1))
        props=[]
        for ln in text.splitlines():
            m=re.match(r'property\s+(\S+)\s+(\S+)',ln)
            if m:props.append((m.group(1),m.group(2)))
        dt=np.dtype([(name,TYPEMAP[typ]) for typ,name in props]);a=np.fromfile(f,dtype=dt,count=n)
        return np.c_[a['x'],a['y'],a['z']].astype(np.float32)

def normalize_sources(src_dir):
    out={}
    for pth in sorted(Path(src_dir).glob('*.ply')):
        p=read_ply_xyz(pth);q=np.c_[p[:,1],p[:,0],p[:,2]].astype(np.float32)
        qlo=np.array([np.quantile(q[:,0],.02),np.quantile(q[:,1],.01),np.quantile(q[:,2],.02)])
        qhi=np.array([np.quantile(q[:,0],.98),np.quantile(q[:,1],.99),np.quantile(q[:,2],.98)])
        q=q-(qlo+qhi)/2;ext=qhi-qlo
        mask=(np.abs(q[:,0])<0.65*ext[0])&(np.abs(q[:,1])<0.60*ext[1])&(np.abs(q[:,2])<0.65*ext[2])
        out[pth.name]=q[mask].astype(np.float32)
    return out

def small_surface_perturb(pts,amp,r):
    p=pts.copy();x,z=p[:,0],p[:,2];rad=np.sqrt(x*x+z*z)+1e-6;theta=np.arctan2(z,x);ph1,ph2=r.uniform(-math.pi,math.pi,2)
    dr=amp*(0.55*np.sin(3*theta+ph1)+0.30*np.sin(7*theta+ph2)+0.15*r.normal(size=len(p)))
    p[:,0]+=dr*x/rad;p[:,2]+=dr*z/rad;p+=r.normal(0,0.010 if amp<=0.02 else 0.014,size=p.shape);return p.astype(np.float32)
def thin_resample(pts,keep,dup,r):
    p=pts[r.random(len(pts))<keep]
    if len(p)==0:return p
    m=int(len(p)*dup)
    if m:p=np.vstack([p,p[r.integers(0,len(p),m)]+r.normal(0,0.008,size=(m,3))])
    return p.astype(np.float32)
def edge_occlude(pts,side,severity,r):
    p=pts;xmin,xmax=np.quantile(p[:,0],[.01,.99]);span=max(xmax-xmin,1e-4);edge_width=min(0.36,0.10+0.42*severity)*span;mask=np.ones(len(p),bool)
    if side in ('left','both'):
        prob=np.clip((p[:,0]-xmin)/(edge_width+1e-6),0,1);zone=p[:,0]<(xmin+edge_width);survive=r.random(len(p))<(0.10+0.75*prob);mask[zone]&=survive[zone]
    if side in ('right','both'):
        prob=np.clip((xmax-p[:,0])/(edge_width+1e-6),0,1);zone=p[:,0]>(xmax-edge_width);survive=r.random(len(p))<(0.10+0.75*prob);mask[zone]&=survive[zone]
    out=p[mask];c=(p[:,0]>(xmin+0.38*span))&(p[:,0]<(xmin+0.62*span));central=p[c]
    if len(central)>20 and np.sum((out[:,0]>(xmin+0.38*span))&(out[:,0]<(xmin+0.62*span)))<0.70*len(central):out=np.vstack([out,central[r.random(len(central))<0.85]])
    return out.astype(np.float32)
def top_filter(p,r):
    z=p[:,2];lo,hi=np.quantile(z,[.02,.98]);zn=np.clip((z-lo)/(hi-lo+1e-6),0,1);return p[r.random(len(p))<(0.45+0.50*zn)]
def side_filter(p,r):
    x=p[:,0];lo,hi=np.quantile(x,[.02,.98]);xn=np.clip((x-lo)/(hi-lo+1e-6),0,1);return p[r.random(len(p))<(0.35+0.58*xn)]
def pose(p,cx,cy,cz,yaw):
    a=math.radians(yaw);c,s=math.cos(a),math.sin(a);q=p.copy();x=q[:,0]*c-q[:,1]*s;y=q[:,0]*s+q[:,1]*c;q[:,0]=x+cx;q[:,1]=y+cy;q[:,2]+=cz;return q
def write_ply(path,xyz,col,gid,iid,layer,slot,vis):
    n=len(xyz);dt=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('global_id','<i4'),('instance_id','<i4'),('layer','u1'),('slot','<i2'),('visibility','u1')]);a=np.empty(n,dtype=dt)
    for k,v in [('x',xyz[:,0]),('y',xyz[:,1]),('z',xyz[:,2])]:a[k]=v
    a['red']=col[:,0];a['green']=col[:,1];a['blue']=col[:,2];a['global_id']=gid;a['instance_id']=iid;a['layer']=layer;a['slot']=slot;a['visibility']=vis
    hdr=('ply\nformat binary_little_endian 1.0\ncomment generated_from_real_coil_prototypes_v3 seed=20260818\ncomment rolling_observation_change_has_no_large_contiguous_wedge_deletion\ncomment partial_occlusion_removes_left_right_edge_regions_not_random_middle_regions\ncomment rgb_is_visualization_only_do_not_use_for_matching\n'+f'element vertex {n}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty int global_id\nproperty int instance_id\nproperty uchar layer\nproperty short slot\nproperty uchar visibility\nend_header\n')
    with open(path,'wb') as f:f.write(hdr.encode('ascii'));a.tofile(f)
def num(x,d=0):
    try:return float(x)
    except:return d

def main(root,out):
    root=Path(root);out=Path(out);out.mkdir(parents=True,exist_ok=True)
    tmp=out/'_src_tmp';
    if tmp.exists():shutil.rmtree(tmp)
    tmp.mkdir()
    with zipfile.ZipFile(root/'assets'/'source_real_coils.zip') as z:z.extractall(tmp)
    protos=normalize_sources(tmp/'test');shutil.rmtree(tmp)
    with open(root/'assets'/'scene_spec.csv','r',encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
    byfile={}
    for r in rows:byfile.setdefault(r['file'],[]).append(r)
    for fn,sr in sorted(byfile.items()):
        X=[];C=[];G=[];I=[];L=[];S=[];V=[]
        for row in sr:
            h=int(hashlib.sha256((fn+'|'+str(row['global_id'])+'|v3').encode()).hexdigest()[:16],16)%(2**63-1);rr=np.random.default_rng(h)
            vis=row['visibility'];obs=row['observation'];sev=num(row['severity'],0.0)
            if vis=='OCCLUDED':p=np.empty((0,3),np.float32)
            else:
                p=protos[row['source_prototype']].copy()
                if obs=='roll_mild':p=thin_resample(small_surface_perturb(p,0.015,rr),0.92,0.06,rr)
                elif obs=='roll_heavy':p=thin_resample(small_surface_perturb(p,0.030,rr),0.79,0.10,rr)
                elif obs=='top_view':p=small_surface_perturb(top_filter(p,rr),0.008,rr)
                elif obs=='side_view':p=small_surface_perturb(side_filter(p,rr),0.010,rr)
                elif obs=='degraded':p=thin_resample(small_surface_perturb(p,0.022,rr),0.82,0.05,rr)
                else:p=(p+rr.normal(0,0.008,size=p.shape)).astype(np.float32)
                if vis=='PARTIAL_VISIBLE' or obs in ('partial','degraded'):
                    side=row.get('occlusion_side','none');s=sev if sev>0 else 0.30;p=edge_occlude(p,side,s,rr);p=thin_resample(p,0.94 if s<0.35 else 0.88,0.02,rr)
            if len(p):p=pose(p,num(row['center_x']),num(row['center_y']),num(row['center_z']),num(row['yaw_deg']))
            gid=int(num(row['global_id']));iid=int(num(row['instance_id'],-1));layer=int(num(row['layer']));slot=int(num(row['slot']));vc={'VISIBLE':1,'PARTIAL_VISIBLE':2,'OCCLUDED':3}.get(vis,0)
            if len(p):
                X.append(p.astype(np.float32));C.append(np.tile(PALETTE[gid%len(PALETTE)],(len(p),1)));G.append(np.full(len(p),gid,np.int32));I.append(np.full(len(p),iid,np.int32));L.append(np.full(len(p),layer,np.uint8));S.append(np.full(len(p),slot,np.int16));V.append(np.full(len(p),vc,np.uint8))
        if X:xyz=np.vstack(X);col=np.vstack(C);gid=np.concatenate(G);iid=np.concatenate(I);layer=np.concatenate(L);slot=np.concatenate(S);vis=np.concatenate(V)
        else:xyz=np.empty((0,3),np.float32);col=np.empty((0,3),np.uint8);gid=np.empty(0,np.int32);iid=np.empty(0,np.int32);layer=np.empty(0,np.uint8);slot=np.empty(0,np.int16);vis=np.empty(0,np.uint8)
        write_ply(out/fn,xyz,col,gid,iid,layer,slot,vis)
    print(f'generated {len(byfile)} PLY files in {out}')
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--root',default=str(Path(__file__).resolve().parents[1]));ap.add_argument('--out',required=True);a=ap.parse_args();main(a.root,a.out)
