#!/usr/bin/env python3
"""English figures/video from recorded simulation, never a hardware recording."""
import json
import hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import imageio_ffmpeg

ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT/'c6_drive'
PORT=BOARD/'portfolio'
files=['encoder-servo-holdout-02.json','encoder-servo-wheels-final.json','encoder-servo-physics-final.json']
reports=[json.loads((BOARD/'reports'/f).read_text()) for f in files]
for report in reports:
    for name,digest in report['source_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest, f'Stale evidence: {name}'
holdout,wheels,physics=reports
colors=['#a96138','#007e79']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':15,'axes.spines.top':False,'axes.spines.right':False,'svg.fonttype':'none'})

summary={}
for group in ['wheel','floor','wall']:
    summary[group]={}
    for variant in ['hold6','servo']:
        rows=[r for r in holdout['results'] if r['variant']==variant and r['case']['name'].startswith(group)]
        summary[group][variant]=dict(n=len(rows),passed=sum(r['metrics']['within_motion_bounds'] for r in rows),
          ended=sum(r['metrics']['ended'] for r in rows),faulted=sum(bool(r['metrics']['faults']) for r in rows),
          mean_cross_mm=float(np.mean([r['metrics']['cross_track_peak_mm'] for r in rows])),
          mean_final_mm=float(np.mean([r['metrics']['final_task_error_mm'] for r in rows])),
          failed_cases=[r['case']['name'] for r in rows if not r['metrics']['within_motion_bounds']])
(BOARD/'reports/encoder-servo-summary.json').write_text(json.dumps(summary,indent=2))

fig,axes=plt.subplots(2,1,figsize=(10,10),layout='constrained')
for ax,metric,label,limit in zip(axes,['cross_track_peak_mm','final_task_error_mm'],
                                 ['Peak sideways deviation (mm)','Final task-position error (mm)'],[5,10]):
    arrays=[]
    for variant in ['hold6','servo']:
        arrays.append([r['metrics'][metric] for r in holdout['results'] if r['variant']==variant and r['case']['name'].startswith('floor')])
    for n in range(len(arrays[0])):ax.plot([0,1],[arrays[0][n],arrays[1][n]],color='#bdc4c9',alpha=.55,lw=1)
    for i,values in enumerate(arrays):
        ax.scatter(np.full(len(values),i),values,s=42,color=colors[i],alpha=.75,zorder=3)
        mean=np.mean(values);ax.plot([i-.13,i+.13],[mean,mean],color='black',lw=3,zorder=4)
        ax.text(i+.17,mean,f'Mean {mean:.2f}',va='center',fontsize=14)
    ax.axhline(limit,ls='--',lw=1,color='#697783',label=f'Declared limit: {limit} mm')
    ax.set(xticks=[0,1],xticklabels=['Previous encoder hold','Continuous servo candidate'],xlim=(-.4,1.75),ylabel=label,ylim=(0,max(limit*1.1,max(map(max,arrays))*1.2)))
    ax.grid(axis='y',alpha=.15);ax.legend(loc='upper right',fontsize=12)
fig.suptitle('Straight-line control | 24 unseen floor trials',fontsize=23,weight='bold')
fig.supxlabel('MuJoCo simulation · Encoder-only C++ control · Paired cases\nLoad, grip and per-wheel delays vary. No physical robot validation.',fontsize=13)
fig.savefig(PORT/'encoder-servo-floor.png',dpi=150);fig.savefig(PORT/'encoder-servo-floor.svg');plt.close(fig)

r=next(r for r in physics['results'] if r['case']['name']=='wall_hold' and r['variant']=='servo')
frames=r['trace'];t=np.array([f['t'] for f in frames]);truth=np.array([f['truth'] for f in frames]);estimate=np.array([f['estimate'] for f in frames])
fig,ax=plt.subplots(figsize=(10,6.3),layout='constrained')
ax.plot(t,truth[:,0]*1000,color='#a96138',lw=3,label='Actual body position in simulation')
ax.plot(t,estimate[:,0]*1000,color='#007e79',lw=3,ls='--',label='Position inferred from encoders')
ax.axhline(0,color='#556472',lw=1)
ax.set(xlabel='Time (s)',ylabel='Vertical displacement (mm)',title='Wall hold: encoder error can hide body sliding')
ax.legend(fontsize=12,loc='lower left');ax.grid(alpha=.15)
fig.supxlabel('Candidate still applies torque. Simulated truth is used only for scoring.\nContact and magnetic force are assumed; this is not a wall-ready controller.',fontsize=12)
fig.savefig(PORT/'encoder-servo-wall.png',dpi=150);fig.savefig(PORT/'encoder-servo-wall.svg');plt.close(fig)

# Readable animated evidence, sourced from an explicitly selected development case.
pair=[next(r for r in physics['results'] if r['case']['name']=='floor_delays' and r['variant']==v) for v in ['hold6','servo']]
fig,axes=plt.subplots(2,1,figsize=(10,7.2),layout='constrained')
artists=[]
for ax,result,color,title in zip(axes,pair,colors,['Previous encoder hold','Continuous servo candidate']):
    tr=np.array([f['truth'][:2] for f in result['trace']])*1000
    ax.plot([0,170],[0,0],ls='--',color='#687782',lw=1,label='Requested straight line')
    line,=ax.plot([],[],color=color,lw=3,label='Simulated chassis path')
    dot,=ax.plot([],[],'o',color=color,ms=9)
    text=ax.text(.02,.92,'',transform=ax.transAxes,va='top',fontsize=14)
    ax.set(xlim=(-5,175),ylim=(-4,4),xlabel='Forward displacement (mm)',ylabel='Sideways (mm)',title=title)
    ax.grid(alpha=.15);artists.append((line,dot,text,tr,result['trace']))
fig.suptitle('Unequal wheel delays | Simulation replay',fontsize=23,weight='bold')
fig.supxlabel('C++ controller reads only encoders and applied PWM. Sideways scale is magnified.\nAssumed delays: 0 / 30 / 70 ms · No hardware footage',fontsize=12)
def update(n):
    for line,dot,text,path,trace in artists:
        k=min(n,len(path)-1);line.set_data(path[:k+1,0],path[:k+1,1]);dot.set_data([path[k,0]],[path[k,1]])
        text.set_text(f"t = {trace[k]['t']:.2f} s   |   sideways = {path[k,1]:+.2f} mm")
    return []
ani=FuncAnimation(fig,update,frames=len(pair[0]['trace']),interval=50,blit=False)
plt.rcParams['animation.ffmpeg_path']=imageio_ffmpeg.get_ffmpeg_exe()
ani.save(PORT/'encoder-servo-comparison.mp4',writer=FFMpegWriter(fps=20,codec='libx264',extra_args=['-pix_fmt','yuv420p','-crf','21']),dpi=120)
plt.close(fig)
print(json.dumps(summary,indent=2))
print('Wrote English PNG/SVG figures and simulation MP4')
