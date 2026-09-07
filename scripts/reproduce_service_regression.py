#!/usr/bin/env python3
"""Demonstrate the early-service defect in an isolated, deliberately mutated build."""
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='fleet-service-regression-') as temp:
    dest=Path(temp)
    for folder in ['src','include','maps']:
        shutil.copytree(root/folder,dest/folder)
    shutil.copy2(root/'CMakeLists.txt',dest/'CMakeLists.txt')
    source=dest/'src/simulator.cpp'
    before=source.read_text()
    marker='if (r.task < 0 || paused[r.id] || r.path.size() > 1)\n        return;'
    if before.count(marker)!=1:
        raise SystemExit('guard changed; review reproducer')
    source.write_text(before.replace(marker, 'if (r.task < 0 || paused[r.id])\n        return;'))
    subprocess.run(['cmake','-S',str(dest),'-B',str(dest/'build'),'-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF'],check=True,stdout=subprocess.DEVNULL)
    subprocess.run(['cmake','--build',str(dest/'build'),'-j2'],check=True,stdout=subprocess.DEVNULL)
    args=['simulate','--map',str(root/'maps/development.map'),'--agents','4','--ticks','60','--seed','91','--strategy','distance','--lambda','0']
    faulty=subprocess.run([str(dest/'build/fleet'),*args],capture_output=True,text=True,check=False)
    fixed=subprocess.run([str(root/'build/fleet'),*args],capture_output=True,text=True,check=True)
    assert faulty.returncode!=0 and 'conflicting reservation input' in faulty.stderr
    summary=json.loads(fixed.stdout)
    print(json.dumps({'status':'reproduced_and_fixed','mutation':'remove reserved-endpoint guard from arrive()',
                      'faulty_exit_code':faulty.returncode,'faulty_error':faulty.stderr.strip(),
                      'fixed_completed':summary['completed'],'fixed_conflicts':summary['conflicts'],
                      'reproducer':'python3 scripts/reproduce_service_regression.py',
                      'environment':'local C++ Release builds; deliberately mutated copy in TemporaryDirectory'},indent=2))
