import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

BINARY = str(Path(sys.argv.pop(1)).resolve())
ROOT = Path(sys.argv.pop(1))
sys.path.insert(0, str(ROOT/'benchmarks'))
from verify import verify_run

class OracleFaults(unittest.TestCase):
    def test_rejects_tampered_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'run'
            subprocess.run([BINARY,'simulate','--map',str(ROOT/'maps/development.map'),
                            '--agents','3','--ticks','30','--out',str(out)],check=True,capture_output=True)
            cfg = {'agents':3,'ticks':30}
            verify_run(ROOT/'maps/development.map',out,cfg)
            summary = (out/'summary.json').read_text()
            result = json.loads(summary)
            result['completed'] += 1
            (out/'summary.json').write_text(json.dumps(result))
            with self.assertRaises(ValueError):verify_run(ROOT/'maps/development.map',out,cfg)
            (out/'summary.json').write_text(summary)
            original = (out/'trace.csv').read_text()
            (out/'trace.csv').write_text('\n'.join(original.splitlines()[:-1])+'\n')
            with self.assertRaises(ValueError):verify_run(ROOT/'maps/development.map',out,cfg)
            (out/'trace.csv').write_text(original)
            rows = list(csv.reader(original.splitlines()))
            rows[2][2] = rows[1][2]
            with (out/'trace.csv').open('w') as f:csv.writer(f).writerows(rows)
            with self.assertRaises(ValueError):verify_run(ROOT/'maps/development.map',out,cfg)
            (out/'trace.csv').write_text(original)
            (out/'COMPLETE').unlink()
            with self.assertRaises(FileNotFoundError):verify_run(ROOT/'maps/development.map',out,cfg)

if __name__=='__main__':unittest.main()
