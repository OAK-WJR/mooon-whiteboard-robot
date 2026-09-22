#!/usr/bin/env python3
"""Reproducible English figures, recorded-physics clips and a small web data set."""
import argparse
import hashlib
import json
import zipfile
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import imageio_ffmpeg
from PIL import Image, ImageDraw
from engine import ROOT, Plant
from render import text, font, INK, MUTED, GREEN, BLUE, ORANGE
from evaluate_robust import audit

OUT = ROOT/'portfolio/robust'
TITLES = {
    'floor': 'Lateral motion and reversal', 'load': 'Unknown external force',
    'payload': 'Heavier payload', 'weak': 'One weaker motor', 'sag': 'Supply sag',
    'saturation': 'Insufficient drive authority', 'slope': 'Inclined board',
    'wall': 'Vertical board and position hold', 'slip': 'Friction loss: physical limit',
    'camera': 'Body feedback lost', 'encoder': 'Encoder marked invalid',
    'magnet': 'Magnetic attraction lost', 'outlier': 'Rejecting a pose outlier',
    'heartbeat': 'Command heartbeat lost', 'stuck': 'An encoder stops reporting motion',
    'diagonal': 'Translation and rotation together',
}
NOTES = {
    'floor': 'Right, hold, left, hold. The three wheel models differ; the common ramp preserves target ratios.',
    'load': 'An unknown -0.8 N board-axis force acts from 3 to 6 s. The controller receives only measured response.',
    'payload': 'Mass is increased in the physical model. It is not supplied to the controller.',
    'weak': 'Motor 1 has 65% of the nominal torque scale; all three wheel demands remain coordinated.',
    'sag': 'Effective supply drops to 65% at 3 s. This is response compensation, not measured-voltage feedforward.',
    'saturation': 'Supply drops to 30% at 3 s. Motion demand eventually reaches zero; the original task is not completed.',
    'slope': 'Gravity acts along the inclined surface. Independent body feedback supplies the position-hold reference.',
    'wall': 'The board is vertical. Gravity acts toward -Y. Magnetic force and torque are assumed, not hardware measurements.',
    'slip': 'Friction drops to 10% at 4 s. The robot falls before this detector trips; the harness freezes at its test boundary.',
    'camera': 'Pose capture stops at 4 s. A stale-body fault clears output, then the robot slides. Zero output is not a wall brake.',
    'encoder': 'An explicit invalid-encoder flag arrives at 4 s. Output is latched off; automatic wire-fault diagnosis is not implied.',
    'magnet': 'Attraction falls to 12.5% at 4 s. Motor PWM cannot restore magnetic force; the simulated robot falls.',
    'outlier': 'One body-pose sample jumps 30 cm at 3.5 s. The C++ plausibility gate rejects it without moving the hold reference.',
    'heartbeat': 'Command timestamps stop advancing at 3.5 s. The controller latches zero output after the freshness limit.',
    'stuck': 'Motor 1 encoder motion is forced to zero at 4 s. Independent body motion exposes inconsistent feedback; a fault latches.',
    'diagonal': 'Diagonal translation and a 0.25 rad/s yaw request reverse together. Scoring retains the original yaw target.',
}
CLIPS = ['floor', 'wall', 'saturation', 'slip']


def load():
    traces = json.loads((ROOT/'reports/robust-nominal-traces.json').read_text())
    reports = {s: json.loads((ROOT/f'reports/robust-{s}.json').read_text()) for s in ['holdout', 'stress']}
    return traces, reports, audit()


def export_data(traces, reports, validation):
    fields = ['t', 'pose', 'task_goal', 'error_m', 'heading_error_deg', 'pwm', 'status', 'fault', 'scale']
    data = {'validation': validation, 'cases': {}, 'pairs': {}}
    for name in TITLES:
        data['cases'][name] = {'title': TITLES[name], 'note': NOTES[name], 'clip': name in CLIPS}
        for r in traces:
            if r['case']['name'] == name and r['variant'] in ['eso', 'robust']:
                data['cases'][name][r['variant']] = {
                    'case': r['case'], 'metrics': r['metrics'],
                    'trace': [{k: f[k] for k in fields} for f in r['trace']],
                }
    for suite, report in reports.items():
        names = sorted({r['case']['name'] for r in report['results']})
        data['pairs'][suite] = [dict(name=n, **{r['variant']: r['metrics'] for r in report['results'] if r['case']['name'] == n}) for n in names]
    (OUT/'data.json').write_text(json.dumps(data, separators=(',', ':'))+'\n')


def plot_campaign(reports, validation):
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11, 'svg.fonttype': 'none'})
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.9), layout='constrained')
    fig.suptitle('WhiteBoardBot | Prospective qualification and retained stress failures', fontsize=19, weight='bold')
    for ax, suite, title in zip(axes, ['holdout', 'stress'], ['60 paired holdouts: declared operating range', '30 wider stress pairs: outside qualification']):
        rows = reports[suite]['results']
        for v, c, label in [('eso', '#427bb8', 'Previous ESO supervisor'), ('robust', '#188a6f', 'Robust C++ candidate')]:
            trials = sorted((r for r in rows if r['variant'] == v), key=lambda r:r['case']['seed'])
            ax.plot([r['case']['seed'] for r in trials], [r['metrics']['mean_task_error_m']*1000 for r in trials], marker='o', ms=3.5, lw=1.3, color=c, label=label)
        ax.set(title=title, xlabel='Random seed (paired fixture and sensor noise)', ylabel='Mean task-position error / mm')
        ax.set_ylim(bottom=0);ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
        ax.ticklabel_format(useOffset=False, axis='x');ax.legend(loc='upper left', fontsize=10)
        summary = validation['suites'][suite]
        caption = f"{summary['eso']['mean_task_error_mm']:.2f} → {summary['robust']['mean_task_error_mm']:.2f} mm mean"
        ax.text(.03,.75,caption,transform=ax.transAxes,fontsize=12,weight='bold')
        if suite == 'holdout':
            ax.set_ylim(0,25)
            ax.text(.03,.66,'60/60 improved · no faults or falls\n52.8% lower mean position error',transform=ax.transAxes,fontsize=10)
        else:
            ax.annotate('STALL → zero output → fall\nSeed 9129; retained in score',xy=(9129,672.66),xytext=(9115,400),arrowprops={'arrowstyle':'->','color':'#b44330'},color='#b44330',fontsize=10)
            ax.text(.03,.66,'Worse mean position error\n1/30 candidate runs fell',transform=ax.transAxes,fontsize=10,color='#b44330')
    fig.supxlabel('Simulation only · 1–8 s error against the original task · assumed contact, torque and body sensing · no hardware deployment',fontsize=10)
    fig.savefig(OUT/'qualification.png',dpi=160)
    fig.savefig(OUT/'qualification.svg')
    plt.close(fig)


def compose(rgb, name, both, frame, index):
    im = Image.new('RGB', (1920,1080), '#0b1118');d = ImageDraw.Draw(im)
    text(d,(64,32),'WHITEBOARD BOT  /  ROBUST C++ CONTROLLER',22,GREEN,True)
    text(d,(1460,34),'SIMULATION  /  NOT FLASHED',20,ORANGE,True)
    text(d,(64,88),TITLES[name],46,INK,True)
    text(d,(64,158),'Native C++ control  ·  MuJoCo contact dynamics  ·  recorded states at 20 Hz',25,MUTED)
    d.rounded_rectangle((64,216,1232,944),16,fill='#121e2a',outline='#26394b',width=2)
    d.rounded_rectangle((1260,216,1856,944),16,fill='#121e2a',outline='#26394b',width=2)
    im.paste(Image.fromarray(rgb),(72,246))
    text(d,(94,221),'CANDIDATE PHYSICS REPLAY',18,GREEN,True)
    text(d,(94,902),'Camera follows the body. The chart uses the original task reference.',23,MUTED)
    text(d,(1300,247),'CONTROLLER STATE',18,MUTED,True)
    text(d,(1300,280),frame['status'].replace('_',' '),29,ORANGE if frame['fault'] or frame['scale']<.98 else GREEN,True)
    text(d,(1300,338),f"Time {frame['t']:.2f} s  |  Motion scale {frame['scale']:.0%}",23)
    text(d,(1300,378),'PWM  '+' / '.join(f'{int(p):+d}' for p in frame['pwm']),23)
    text(d,(1300,439),'Mean task error, 1–8 s',25,INK,True)
    for y,v,label,col in [(486,'eso','Previous ESO',BLUE),(537,'robust','Robust C++',GREEN)]:
        text(d,(1300,y),label,24,col)
        text(d,(1655,y-5),f"{both[v]['metrics']['mean_task_error_m']*1000:.1f} mm",30,col,True)
    text(d,(1300,598),'POSITION ERROR / mm',18,MUTED,True)
    left,top,w,h=1350,652,450,192
    maximum=max(35,max(f['error_m']*1000 for r in both.values() for f in r['trace']))*1.08
    for j in range(4):
        yy=top+j*h/3;d.line((left,yy,left+w,yy),fill='#304455')
        text(d,(1296,yy-8),f'{maximum*(1-j/3):.0f}',16,MUTED)
    for v,col in [('eso',BLUE),('robust',GREEN)]:
        pts=[(left+f['t']/8*w,top+h-f['error_m']*1000/maximum*h) for f in both[v]['trace'][:index+1]]
        if len(pts)>1:d.line(pts,fill=col,width=3)
    text(d,(left,859),'0 s',17,MUTED);text(d,(left+w-28,859),'8 s',17,MUTED)
    text(d,(1300,901),'Output off does not prevent a wall fall.',22,ORANGE)
    # Wrap the scenario-specific disclosure to avoid clipping long English captions.
    words=NOTES[name].split();lines=['']
    for word in words:
        candidate=(lines[-1]+' '+word).strip()
        if d.textbbox((0,0),candidate,font=font(23))[2]>1780:lines.append(word)
        else:lines[-1]=candidate
    for j,line in enumerate(lines):text(d,(64,967+j*33),line,23,MUTED)
    text(d,(64,1040),'Model assumptions remain uncalibrated: mass, traction, torque, adhesion and independent body sensing.',20,'#73899d')
    return im


def render_clips(traces):
    meta=[]
    for name in CLIPS:
        both={r['variant']:r for r in traces if r['case']['name']==name and r['variant'] in ['eso','robust']}
        record=both['robust'];cfg=record['case'];trace=record['trace']
        plant=Plant(mass=cfg['mass'],mu=cfg['mu'],incline=cfg['incline'])
        renderer=mujoco.Renderer(plant.m,height=648,width=1152)
        cam=mujoco.MjvCamera();cam.distance=.26;cam.azimuth=115;cam.elevation=-52
        writer=imageio_ffmpeg.write_frames(str(OUT/f'{name}.mp4'),(1920,1080),fps=20,quality=8,
            macro_block_size=8,codec='libx264',output_params=['-movflags','+faststart'])
        writer.send(None)
        try:
            for i in range(200):
                index=min(len(trace)-1,max(0,i-20));f=trace[index]
                plant.d.qpos[:]=f['qpos'];mujoco.mj_forward(plant.m,plant.d)
                cam.lookat[:]=plant.d.xpos[plant.body];cam.lookat[2]=.008
                renderer.update_scene(plant.d,camera=cam)
                im=compose(renderer.render(),name,both,f,index)
                writer.send(np.asarray(im))
                if i==130:im.save(OUT/f'{name}.png')
        finally:
            writer.close();renderer.close()
        # Full decoding verifies the generated artifact, not just its container.
        reader=imageio_ffmpeg.read_frames(str(OUT/f'{name}.mp4'),pix_fmt='rgb24')
        info=next(reader);count=sum(1 for _ in reader)
        if info['size']!=(1920,1080) or count!=200:raise RuntimeError(f'Invalid capture: {name}: {info}, {count}')
        meta.append({'case':name,'title':TITLES[name],'seconds':10,'fps':20,'decoded_frames':count,'resolution':[1920,1080]})
        print('Rendered and decoded',name,flush=True)
    return meta


def package_media():
    archive=OUT.parent/'robust-controller-study.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(OUT.iterdir()):
            if p.is_file():z.write(p,'robust/'+p.name)
        z.write(ROOT/'ROBUST_CONTROL.md','ROBUST_CONTROL.md')
        for name in ['robust-validation.json','robust-holdout.json','robust-stress.json','robust-nominal.json']:
            z.write(ROOT/'reports'/name,'reports/'+name)
        z.writestr('START_HERE.md', '# English portfolio media\n\nScreenshots, plots, videos and capture settings are in robust/. Complete results are in reports/. All media show simulation, not the physical robot. The interactive gallery runs from the WhiteBoardBot repository at http://127.0.0.1:8088/portfolio/robust.html after starting scripts/c6_drive_sim.py; it is not an offline page in this media pack.\n')
    print('Saved',archive,flush=True)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--figures-only',action='store_true');args=ap.parse_args()
    OUT.mkdir(exist_ok=True);traces,reports,validation=load()
    export_data(traces,reports,validation);plot_campaign(reports,validation)
    if args.figures_only:
        if (OUT/'capture-manifest.json').exists():package_media()
        return
    clips=render_clips(traces)
    manifest={'simulation_only':True,'language':'en','mujoco':mujoco.__version__,
              'method':'Native MuJoCo render of saved C++-controlled states; 1 s initial still, 8 s recorded playback, 1 s final still. No interpolation or trajectory retouching.',
              'clips':clips,'controller_sha256':validation['source_sha256']['include/robust_control.h'],
              'trace_sha256':hashlib.sha256((ROOT/'reports/robust-nominal-traces.json').read_bytes()).hexdigest()}
    (OUT/'capture-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (OUT/'README.md').write_text('''# WhiteBoardBot: robust controller study

English portfolio material. All footage is simulation, not a recording of the physical robot.

The portable C++ core is executed directly in MuJoCo. The 60 prospective holdouts average 9.417 mm error with the previous ESO supervisor and 4.447 mm with the candidate. The separate wider stress set is worse in mean position error and includes one stall followed by a fall. See qualification.png and the complete reports; do not present only successful cases as universal robustness.

Clips: floor (coordinated reversal), wall (position hold with an assumed pose sensor), saturation (demand falls to zero, task incomplete), slip (insufficient traction, fall before detection). Each clip is 1920×1080, H.264, 20 fps, 10 seconds, silent. The start/end each freeze for one second. The renderer follows the robot; side plots show error from the original requested task.

Measured: suspended moving-range motor dynamics. Assumed: mass, torque scale, inertia, passive-roller geometry, friction, magnetic force and body sensor. No hardware deployment or validated fall prevention. Original programmatic geometry; rendered using MuJoCo. No third-party robot meshes or stock footage.

Reproduce: .venv/bin/python c6_drive/sim/physics/render_robust.py
Implementation and primary references: ../../ROBUST_CONTROL.md
''')
    package_media()


if __name__=='__main__':main()
