"""Minimal binary little-endian PLY reader for this dataset.
IMPORTANT: load_algorithm_input() intentionally hides GT-only fields.
"""
from pathlib import Path
import re
import numpy as np

TYPEMAP={'float':'<f4','float32':'<f4','double':'<f8','uchar':'u1','uint8':'u1','char':'i1','short':'<i2','ushort':'<u2','int':'<i4','uint':'<u4'}

def read_ply(path):
    path=Path(path)
    with path.open('rb') as f:
        header=[]
        while True:
            line=f.readline()
            if not line: raise ValueError(f'invalid PLY: {path}')
            header.append(line)
            if line.strip()==b'end_header': break
        text=b''.join(header).decode('ascii','ignore')
        fmt=re.search(r'format\s+(\S+)',text).group(1)
        if fmt!='binary_little_endian': raise ValueError(f'unsupported PLY format: {fmt}')
        n=int(re.search(r'element vertex\s+(\d+)',text).group(1))
        props=[]
        for ln in text.splitlines():
            m=re.match(r'property\s+(\S+)\s+(\S+)',ln)
            if m: props.append((m.group(1),m.group(2)))
        dt=np.dtype([(name,TYPEMAP[typ]) for typ,name in props])
        arr=np.fromfile(f,dtype=dt,count=n)
    return arr

def load_algorithm_input(path):
    """Return only legal algorithm inputs: xyz + per-frame instance_id.
    Do NOT expose RGB/global_id/layer/slot/visibility from the PLY.
    """
    a=read_ply(path)
    xyz=np.column_stack([a['x'],a['y'],a['z']]).astype(np.float32)
    instance_id=a['instance_id'].astype(np.int32)
    return xyz, instance_id

def split_instances(path):
    xyz,iid=load_algorithm_input(path)
    return {int(k):xyz[iid==k] for k in np.unique(iid)}
