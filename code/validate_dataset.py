from pathlib import Path
import csv
ROOT=Path(__file__).resolve().parents[1]
with open(ROOT/'labels'/'object_gt.csv','r',encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
by={}
for r in rows:by.setdefault(r['file'],[]).append(r)
errors=[]
for fn,rs in by.items():
    if sum(int(float(r['layer']))==1 for r in rs)>10:errors.append(fn+': layer1 > 10')
    for r in rs:
        if r['visibility']=='PARTIAL_VISIBLE' and r['occlusion_side'] not in ('left','right','both'):errors.append(fn+': bad partial side')
ply=list(ROOT.glob('*.ply'))
if len(ply)!=68:errors.append(f'expected 68 root PLYs, got {len(ply)}')
if list(ROOT.glob('*/*.ply')):errors.append('nested PLY exists')
print('PASS' if not errors else 'FAIL')
for e in errors:print(e)
