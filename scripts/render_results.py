#!/usr/bin/env python3
"""Render scientific comparison and a trace-derived fault replay (no invented robot positions)."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

p=argparse.ArgumentParser()
p.add_argument('--results',type=Path,required=True)
p.add_argument('--demo',type=Path,required=True)
p.add_argument('--map',type=Path,required=True)
p.add_argument('--out',type=Path,default=Path('docs/assets'))
a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
summary=json.loads((a.results/'summary.json').read_text())
fig,axes=plt.subplots(1,3,figsize=(12,3.7),sharey=True,layout='constrained')
colors=['#475569','#0284c7','#d97706']
for ax,m in zip(axes,['open','narrow','cross']):
    for i,v in enumerate(['nearest','distance','congestion']):
        groups=sorted([x for x in summary['groups'] if x['map']==m and x['variant']==v],key=lambda x:x['agents'])
        ax.plot([x['agents'] for x in groups],[x['mean_completed'] for x in groups],marker='o',label=v,color=colors[i])
    ax.set_title(m.capitalize());ax.set_xlabel('Simulated robots');ax.set_xticks([10,30,50]);ax.grid(alpha=.2)
axes[0].set_ylabel('Mean completed pickup/delivery tasks / 300 ticks')
axes[-1].legend(frameon=False)
fig.suptitle('Held-out local simulation • 10 paired seeds per map/load • fixed planner',fontsize=12)
fig.savefig(a.out/'comparison.png',dpi=180)
fig.savefig(a.out/'comparison.svg')
svg=a.out/'comparison.svg'
svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
plt.close(fig)
rows=a.map.read_text().splitlines();w,h=len(rows[0]),len(rows);scale=24
frames={}
with (a.demo/'trace.csv').open() as f:
    for r in csv.DictReader(f):frames.setdefault(int(r['tick']),[]).append(r)
events=[json.loads(x) for x in (a.demo/'events.jsonl').read_text().splitlines()]
images=[];active_closed=set();paused=set();completed=0
palette=['#2563eb','#db2777','#059669','#d97706','#7c3aed','#0891b2']
for tick,robots in frames.items():
    # Fault event at tick affects the transition tick->tick+1; display it at its barrier.
    for e in events:
        if e['tick']!=tick:continue
        if e['kind']=='fault_start':(paused if e['detail']=='pause' else active_closed).add(e['robot'])
        if e['kind']=='fault_clear':(paused if e['detail']=='pause' else active_closed).discard(e['robot'])
        if e['kind']=='task_state' and e['detail'].endswith('->Completed'):completed+=1
    im=Image.new('RGB',(w*scale,h*scale+65),'#f8fafc');d=ImageDraw.Draw(im)
    d.text((10,8),f'LOCAL DISCRETE REPLAY | tick {tick:03d} | completed {completed}',fill='#0f172a')
    d.text((10,28),'Red = paused robot / closed cell; positions come from trace.csv',fill='#475569')
    for y,row in enumerate(rows):
        for x,ch in enumerate(row):
            color='#334155' if ch=='#' else '#e2e8f0'
            if y*w+x in active_closed:color='#ef4444'
            d.rectangle((x*scale,y*scale+65,(x+1)*scale-1,(y+1)*scale+64),fill=color)
    for r in robots:
        cell,robot=int(r['cell']),int(r['robot']);x,y=(cell%w)*scale,(cell//w)*scale+65
        color='#ef4444' if robot in paused else palette[robot%len(palette)]
        d.ellipse((x+3,y+3,x+scale-3,y+scale-3),fill=color)
        d.text((x+7,y+6),str(robot),fill='white')
    images.append(im)
images[0].save(a.out/'recovery.gif',save_all=True,append_images=images[1:],duration=100,loop=0,optimize=True)
print(a.out)
