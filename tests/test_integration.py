"""Exercise the real CLI, log protocol, replay validation and policy fairness."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

BINARY = str(Path(sys.argv.pop(1)).resolve())
ROOT = Path(sys.argv.pop(1))

class Integration(unittest.TestCase):
    def run_cli(self, *args, ok=True):
        r = subprocess.run([BINARY, *map(str, args)], text=True, capture_output=True, timeout=30)
        self.assertEqual(r.returncode == 0, ok, r.stderr)
        return r

    def test_replay_and_idempotent_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'run'
            args = ['simulate', '--map', ROOT/'maps/development.map', '--agents', 3, '--ticks', 40, '--out', out]
            self.run_cli(*args)
            original = (out/'trace.csv').read_bytes()
            self.run_cli(*args, ok=False)
            self.assertEqual(original, (out/'trace.csv').read_bytes())
            self.assertTrue((out/'COMPLETE').exists())
            self.run_cli('replay', '--map', ROOT/'maps/development.map', '--trace', out/'trace.csv')
            lines = (out/'trace.csv').read_text().splitlines()
            row0, row1 = lines[1].split(','), lines[2].split(',')
            row1[2] = row0[2]
            lines[2] = ','.join(row1)
            (out/'bad.csv').write_text('\n'.join(lines)+'\n')
            self.run_cli('replay', '--map', ROOT/'maps/development.map', '--trace', out/'bad.csv', ok=False)

    def test_paired_workload_and_zero_lambda(self):
        with tempfile.TemporaryDirectory() as tmp:
            records = []
            for strategy in ['nearest', 'distance', 'congestion']:
                out = Path(tmp)/strategy
                result = json.loads(self.run_cli('simulate', '--map', ROOT/'maps/development.map', '--agents', 4, '--ticks', 60, '--seed', 91, '--strategy', strategy, '--lambda', 0, '--out', out).stdout)
                events = [json.loads(x) for x in (out/'events.jsonl').read_text().splitlines()]
                offers = [x for x in events if x['kind']=='task_offer']
                records.append((result, offers, (out/'trace.csv').read_bytes()))
            self.assertEqual(records[0][1], records[1][1])
            self.assertEqual(records[1][1], records[2][1])
            self.assertEqual(records[1][2], records[2][2])
            self.assertEqual(records[1][0]['completed'], records[2][0]['completed'])

    def test_fault_accounting_and_logs(self):
        with tempfile.TemporaryDirectory() as tmp:
            faults = Path(tmp)/'faults.txt'
            faults.write_text('pause 0 5 8\nclosure 0 20 8\n')
            out = Path(tmp)/'run'
            r = json.loads(self.run_cli('simulate', '--map', ROOT/'maps/development.map', '--agents', 1, '--ticks', 60, '--faults', faults, '--out', out).stdout)
            events = [json.loads(x) for x in (out/'events.jsonl').read_text().splitlines()]
            self.assertTrue(any(x['kind']=='plan_epoch_reset' for x in events))
            self.assertEqual(r['admitted'], r['completed']+r['unfinished'])
            self.assertEqual(r['offered'], r['admitted']+r['rejected'])
            self.assertEqual(r['conflicts'], 0)
            self.assertEqual(len(r['recovery_ticks']), 2)

    def test_invalid_inputs(self):
        for key, value in [('--agents','-1'),('--ticks','0'),('--lambda','nan'),('--horizon','99999'),('--seed','1x'),('--bogus','3')]:
            self.run_cli('simulate','--map',ROOT/'maps/open.map',key,value,ok=False)

if __name__ == '__main__':
    unittest.main()
