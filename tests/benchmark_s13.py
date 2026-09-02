from pathlib import Path
import sys,time,json
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tracker.engine import load_config
from tracker.dataset_adapter import SyntheticDatasetAdapter
from tracker.geometry import observe_frame

def main():
 cfg=load_config(ROOT/'tracker_config.json'); ds=SyntheticDatasetAdapter(ROOT)
 t0=time.perf_counter(); inst=ds.load_instances('S13_F00_full_10_plus_8_100k.ply'); t1=time.perf_counter(); obs=observe_frame(inst,cfg); t2=time.perf_counter()
 d={'file':'S13_F00_full_10_plus_8_100k.ply','instances':len(inst),'points':sum(map(len,inst.values())),'load_ms':(t1-t0)*1000,'geometry_ms':(t2-t1)*1000,'total_ms':(t2-t0)*1000,'target_ms':2000,'pass':(t2-t0)*1000<2000}
 (ROOT/'results/s13_benchmark.json').write_text(json.dumps(d,indent=2),encoding='utf-8');print(json.dumps(d,indent=2))
if __name__=='__main__':main()
