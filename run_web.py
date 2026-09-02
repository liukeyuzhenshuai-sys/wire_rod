from pathlib import Path
import argparse,sys
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import uvicorn

def main():
    p=argparse.ArgumentParser(description='Run the local semantic-tracker inspection web UI')
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=9999);p.add_argument('--reload',action='store_true')
    a=p.parse_args()
    print(f'Open http://{a.host}:{a.port}')
    uvicorn.run('web.app:app',host=a.host,port=a.port,reload=a.reload,app_dir=str(ROOT))
if __name__=='__main__':main()
