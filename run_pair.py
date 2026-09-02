from pathlib import Path
import argparse,json,sys
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
from tracker import run_pair,evaluate_pair,load_config

def main():
 p=argparse.ArgumentParser();p.add_argument('pair_id');p.add_argument('--history-mode',default='confirmed_gt',choices=['confirmed_gt','estimated_geometry']);p.add_argument('--out',default='')
 a=p.parse_args();cfg=load_config(ROOT/'tracker_config.json');r=run_pair(ROOT,a.pair_id,cfg,a.history_mode);e=evaluate_pair(ROOT,a.pair_id,r);d={'run':r,'evaluation':e}
 text=json.dumps(d,indent=2,ensure_ascii=False)
 if a.out: Path(a.out).write_text(text,encoding='utf-8')
 else: print(text)
if __name__=='__main__':main()
