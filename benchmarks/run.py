#!/usr/bin/env python3
"""Bounded isolated-process experiments with timeout, paired inputs and independent validation."""
import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import gzip
import hashlib
import itertools
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time
from verify import verify_run

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(args):
    config = json.loads(args.config.read_text())
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    binary = args.binary.resolve()
    env = {'recorded_at': datetime.now(timezone.utc).isoformat(), 'platform': platform.platform(),
           'logical_cpus': os.cpu_count(), 'cpu_model': next((x.split(':',1)[1].strip() for x in Path('/proc/cpuinfo').read_text().splitlines() if x.startswith('model name')), 'unknown'),
           'memory_kib': int(Path('/proc/meminfo').read_text().splitlines()[0].split()[1]),
           'python': sys.version, 'compiler': subprocess.check_output(['c++','--version'],text=True).splitlines()[0],
           'cmake': subprocess.check_output(['cmake','--version'],text=True).splitlines()[0],
           'binary_sha256': digest(binary), 'workers': args.workers, 'execution_mode': 'local_discrete_cpu_processes',
           'config_sha256': digest(args.config), 'command': sys.argv,
           'map_sha256': {m:digest(ROOT/'maps'/f'{m}.map') for m in config['maps']},
           'source_sha256': {str(p.relative_to(ROOT)):digest(p) for folder in ['src','include','benchmarks'] for p in sorted((ROOT/folder).rglob('*')) if p.is_file() and '__pycache__' not in str(p)}}
    (out/'environment.json').write_text(json.dumps(env,indent=2)+'\n')
    (out/'protocol.json').write_text(json.dumps(config,indent=2)+'\n')
    jobs = list(itertools.product(config['maps'],config['agents'],config['seeds'],config['strategies']))
    # Interleave method order by seed, keeping every exact input paired.
    jobs.sort(key=lambda x:(x[0],x[1],x[2],(config['strategies'].index(x[3])+x[2])%len(config['strategies'])))

    def one(job):
        map_name, agents, seed, variant = job
        run_id = f'{map_name}-a{agents}-s{seed}-{variant}'
        folder = out/run_id
        strategy = 'congestion' if variant=='congestion_zero' else variant
        cmd = [str(binary),'simulate','--map',str(ROOT/'maps'/f'{map_name}.map'),'--agents',str(agents),'--seed',str(seed),'--strategy',strategy,'--lambda',str(0 if variant=='congestion_zero' else config['lambda']),'--out',str(folder)]
        for key, option in [('ticks','ticks'),('horizon','horizon'),('max_expansions','max-expansions'),('deadline_us','deadline-us'),('arrival_interval','arrival-interval'),('max_pending','max-pending'),('congestion_window','window'),('no_progress_ticks','no-progress-ticks')]:
            cmd += ['--'+option,str(config[key])]
        begin = time.perf_counter()
        try:
            process = subprocess.run(cmd, capture_output=True, text=True, timeout=args.timeout)
            if process.returncode:
                raise RuntimeError(process.stderr.strip())
            r = json.loads((folder/'summary.json').read_text())
            verification = verify_run(ROOT/'maps'/f'{map_name}.map',folder,{**config,'agents':agents})
            r.update({'run_id':run_id,'map':map_name,'variant':variant,'status':'passed','process_wall_seconds':time.perf_counter()-begin,'oracle':verification})
            hashes = {p.name:digest(p) for p in folder.iterdir() if p.is_file()}
            r['artifact_sha256'] = hashes
            for name in ['events.jsonl','trace.csv']:
                p = folder/name
                with (folder/(name+'.gz')).open('wb') as raw:
                    with gzip.GzipFile(filename='',mode='wb',fileobj=raw,mtime=0) as zipped:
                        zipped.write(p.read_bytes())
                p.unlink()
            # Detailed arrays remain in the per-run summary, avoid duplicating them in the index.
            for key in ['planning_us','allocation_us','completion_ticks']:
                r.pop(key,None)
            return r
        except (OSError,ValueError,RuntimeError,subprocess.TimeoutExpired) as e:
            return {'run_id':run_id,'map':map_name,'agents':agents,'seed':seed,'variant':variant,'status':'failed','error':str(e)}

    records = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool, (out/'runs.jsonl').open('w') as index:
        for record in pool.map(one,jobs):
            records.append(record)
            index.write(json.dumps(record,sort_keys=True)+'\n');index.flush()
            if len(records)%12==0 or record['status']!='passed':
                print(f'{len(records)}/{len(jobs)} {record["run_id"]}: {record["status"]}',flush=True)
    failures = [x for x in records if x['status']!='passed']
    paired = defaultdict(list)
    for r in records:
        if r['status']=='passed': paired[r['map'],r['agents'],r['seed']].append(r)
    for key, group in paired.items():
        if len({r['oracle']['workload_sha256'] for r in group})!=1:
            failures.append({'pair':key,'error':'workload differs between strategies'})
        variants = {r['variant']:r for r in group}
        if 'distance' in variants and 'congestion_zero' in variants:
            a,b = variants['distance'],variants['congestion_zero']
            if a['artifact_sha256']['trace.csv'] != b['artifact_sha256']['trace.csv']:
                failures.append({'pair':key,'error':'lambda=0 trace differs from distance policy'})
    if digest(binary) != env['binary_sha256']:
        failures.append({'error':'binary changed during experiment; results are ineligible'})
    for path, expected in env['source_sha256'].items():
        if digest(ROOT/path) != expected:
            failures.append({'error':'source changed during experiment','path':path})
    grouped = defaultdict(list)
    for r in records:
        if r['status']=='passed': grouped[r['map'],r['agents'],r['variant']].append(r)
    summary = []
    for (m,n,v), group in sorted(grouped.items()):
        latencies = []
        for r in group:
            latencies += json.loads((out/r['run_id']/'summary.json').read_text())['planning_us']
        latencies.sort()
        baseline = {r['seed']:r for r in records if r.get('status')=='passed' and r['map']==m and r['agents']==n and r['variant']=='nearest'}
        deltas = [r['completed']-baseline[r['seed']]['completed'] for r in group if r['seed'] in baseline]
        summary.append({'map':m,'agents':n,'variant':v,'runs':len(group),
                        'mean_completed':statistics.mean(r['completed'] for r in group),
                        'mean_unfinished':statistics.mean(r['unfinished'] for r in group),
                        'mean_throughput':statistics.mean(r['throughput_tasks_per_tick'] for r in group),
                        'mean_paired_completed_delta_vs_nearest':statistics.mean(deltas) if deltas else None,
                        'min_paired_delta':min(deltas) if deltas else None,'max_paired_delta':max(deltas) if deltas else None,
                        'pooled_planning_us_p95':latencies[max(0,(95*len(latencies)+99)//100-1)] if latencies else None,
                        'conflicts':sum(r['conflicts'] for r in group),
                        'no_progress_events':sum(r['no_progress_events'] for r in group)})
    final = {'status':'failed' if failures else 'passed','runs':len(records),'passed':len(records)-sum(r['status']=='failed' for r in records),
             'failures':failures,'groups':summary,'notes':'Descriptive paired results, no significance or production claim. p95 pools measured planner calls, not per-run p95 averages.'}
    (out/'summary.json').write_text(json.dumps(final,indent=2)+'\n')
    if not failures:(out/'COMPLETE').write_text('verified-v1\n')
    print(json.dumps({k:v for k,v in final.items() if k!='groups'},indent=2))
    return bool(failures)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,default=ROOT/'build/fleet')
    parser.add_argument('--config',type=Path,default=ROOT/'configs/evaluation.json')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--workers',type=int,default=1)
    parser.add_argument('--timeout',type=int,default=120)
    args=parser.parse_args()
    if not 1<=args.workers<=4 or args.timeout<1:parser.error('workers must be 1..4; timeout positive')
    raise SystemExit(run(args))
