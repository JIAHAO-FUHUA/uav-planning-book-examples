"""Retrieve pinned upstream files with SHA-256 verification; no git credentials.

Only the two required catkin source subtrees are downloaded. Existing unrelated
files are untouched. A mismatched source is rejected rather than overwritten.
"""
from pathlib import Path
import argparse,hashlib,json,concurrent.futures,urllib.request,time

def sha(data):return hashlib.sha256(data).hexdigest()

def prepare(root):
    root=Path(root).expanduser().resolve();root.mkdir(parents=True,exist_ok=True)
    manifest=json.loads(Path(__file__).with_name('pinned_sources.json').read_text())
    def retrieve(entry):
        p=root/entry['destination'];p.parent.mkdir(parents=True,exist_ok=True)
        assert p.resolve().is_relative_to(root) if hasattr(p.resolve(),'is_relative_to') else str(p.resolve()).startswith(str(root)+'/')
        if p.exists():
            data=p.read_bytes()
            if entry['destination'].endswith('bspline_opt/CMakeLists.txt'):
                data=data.replace(str(root/'deps/include').encode(),b'/usr/local/include').replace(str(root/'deps/lib/libnlopt.so').encode(),b'/usr/local/lib/libnlopt.so')
            if sha(data)==entry['sha256']:return 'cached'
            raise RuntimeError('Existing source differs; preserve/review it first: '+str(p))
        for attempt in range(5):
            try:
                req=urllib.request.Request(entry['url'],headers={'User-Agent':'uav-planning-book-reproduction'})
                with urllib.request.urlopen(req,timeout=90) as response:data=response.read()
                if sha(data)!=entry['sha256']:raise RuntimeError('SHA-256 mismatch: '+entry['url'])
                temporary=p.with_name(p.name+'.download');temporary.write_bytes(data);temporary.replace(p)
                if entry.get('executable'):p.chmod(0o755)
                return 'downloaded'
            except Exception:
                if attempt==4:raise
                time.sleep(attempt+1)
    with concurrent.futures.ThreadPoolExecutor(max_workers=12) as pool:
        outcomes=list(pool.map(retrieve,manifest['files']))
    print('Pinned source files:',len(outcomes),'downloaded:',outcomes.count('downloaded'))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',default=str(Path.home()/'uav_book_cases_20261020'))
    prepare(p.parse_args().root)
