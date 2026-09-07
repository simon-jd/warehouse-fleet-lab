"""Independent Python trace/event oracle; does not import the C++ planner."""
import csv
import hashlib
import json
from pathlib import Path


def verify_run(map_path, run_path, config):
    rows = Path(map_path).read_text().splitlines()
    width = len(rows[0])
    n = config['agents']
    run = Path(run_path)
    if (run/'COMPLETE').read_text() != 'v1\n':
        raise ValueError('missing completion marker')
    result = json.loads((run/'summary.json').read_text())
    if result['offered'] != result['admitted'] + result['rejected']:
        raise ValueError('invalid admission accounting')
    if result['admitted'] != result['completed'] + result['unfinished']:
        raise ValueError('invalid task accounting')
    task_states, completed, offers = {}, set(), []
    closures = {}
    allowed = {'Pending': {'Assigned'}, 'Assigned': {'Pending', 'InProgress'},
               'InProgress': {'Completed'}, 'Completed': set()}
    events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    last_tick = 0
    for e in events:
        if not last_tick <= e['tick'] <= config['ticks']:
            raise ValueError('unordered event tick')
        last_tick = e['tick']
        task = e['task']
        if e['kind']=='task_offer':
            offers.append(e)
        elif e['kind']=='task_admitted':
            if task in task_states:
                raise ValueError('duplicate task')
            task_states[task] = 'Pending'
        elif e['kind']=='task_state':
            before, after = e['detail'].split('->')
            if task_states.get(task) != before or after not in allowed[before]:
                raise ValueError('illegal task transition')
            task_states[task] = after
            if after == 'Completed':
                completed.add(task)
        if e['kind'] in ['fault_start', 'fault_clear'] and e['detail']=='closure':
            closures.setdefault(e['tick'], []).append((e['kind'], e['robot']))
    if len(completed)!=result['completed'] or len(task_states)!=result['admitted']:
        raise ValueError('event/result task mismatch')
    active_closed = set()
    previous = None
    initial = None
    frames = 0
    with (run/'trace.csv').open() as f:
        reader = csv.DictReader(f)
        while True:
            batch = []
            for robot in range(n):
                row = next(reader, None)
                if row is None:
                    if batch:
                        raise ValueError('truncated frame')
                    break
                if int(row['tick'])!=frames or int(row['robot'])!=robot:
                    raise ValueError('trace order or fleet size')
                batch.append(int(row['cell']))
            if not batch:
                break
            if len(set(batch)) != n:
                raise ValueError('vertex conflict')
            if previous is not None:
                for kind, cell in closures.get(frames-1, []):
                    if kind=='fault_start': active_closed.add(cell)
                    else: active_closed.discard(cell)
            for robot, cell in enumerate(batch):
                if not 0<=cell<width*len(rows) or rows[cell//width][cell%width]!='.':
                    raise ValueError('invalid map cell')
                if cell in active_closed:
                    raise ValueError('closed cell occupied')
                if previous is not None:
                    old = previous[robot]
                    if abs(old//width-cell//width)+abs(old%width-cell%width)>1:
                        raise ValueError('illegal movement')
                    for other in range(robot):
                        if previous[other]==cell and batch[other]==old:
                            raise ValueError('edge swap')
            if initial is None:
                initial = batch
            previous = batch
            frames += 1
    if frames != config['ticks']+1:
        raise ValueError('missing frames')
    workload = json.dumps({'initial': initial, 'offers': offers}, sort_keys=True).encode()
    return {'frames': frames, 'robot_steps': n*(frames-1), 'conflicts': 0,
            'workload_sha256': hashlib.sha256(workload).hexdigest()}
