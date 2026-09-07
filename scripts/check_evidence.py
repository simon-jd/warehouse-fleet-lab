#!/usr/bin/env python3
"""Check the published benchmark archive's file digests and independent run oracles."""
import argparse
import gzip
import hashlib
import json
import sys
import tarfile
import tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'benchmarks'))
from verify import verify_run

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--results',type=Path,default=ROOT/'docs/evidence/held-out')
a=p.parse_args()
records=[json.loads(x) for x in (a.results/'runs.jsonl').read_text().splitlines()]
environment=json.loads((a.results/'environment.json').read_text())
config=json.loads((a.results/'protocol.json').read_text())
for name,expected in environment['source_sha256'].items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected, f'source changed: {name}'
for name,expected in environment['map_sha256'].items():
    assert hashlib.sha256((ROOT/'maps'/f'{name}.map').read_bytes()).hexdigest()==expected
checked=0
with tarfile.open(a.results/'raw-runs.tar.gz','r:gz') as archive, tempfile.TemporaryDirectory(prefix='fleet-evidence-check-') as temp:
    for r in records:
        folder=Path(temp)/r['run_id'];folder.mkdir()
        for name,expected in r['artifact_sha256'].items():
            stored=name+'.gz' if name in ['trace.csv','events.jsonl'] else name
            member=archive.extractfile(r['run_id']+'/'+stored)
            assert member is not None
            body=member.read()
            if stored.endswith('.gz'):body=gzip.decompress(body)
            assert hashlib.sha256(body).hexdigest()==expected, f'digest mismatch: {r["run_id"]}/{name}'
            (folder/name).write_bytes(body)
        actual=verify_run(ROOT/'maps'/f'{r["map"]}.map',folder,{**config,'agents':r['agents']})
        assert actual==r['oracle'],f'oracle mismatch: {r["run_id"]}'
        checked+=1
print(json.dumps({'status':'passed','verified_runs':checked,'source_and_map_hashes':'matched','oracle':'independent Python frame/task checker'},indent=2))
