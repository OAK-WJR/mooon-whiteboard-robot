#!/usr/bin/env python3
"""Render saved MuJoCo states to English portfolio images and H.264 video.

Uses real MuJoCo rendering of computed qpos; no trajectory smoothing or retouching.
The presentation adds legends and a tracking-error comparison to each frame.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import mujoco
import imageio_ffmpeg
from PIL import Image, ImageDraw, ImageFont
from engine import ROOT, Plant, SCENARIOS

OUT = ROOT/'portfolio'
INK, MUTED, GREEN, BLUE, ORANGE = '#e5eef7', '#91a7bc', '#72debd', '#7faeee', '#e1a47b'
FONTS = Path('/System/Library/Fonts/Supplemental')


def font(size, bold=False):
    name = 'Arial Bold.ttf' if bold else 'Arial.ttf'
    path = FONTS/name
    if path.exists():
        return ImageFont.truetype(str(path), size)
    import matplotlib.font_manager
    return ImageFont.truetype(matplotlib.font_manager.findfont('DejaVu Sans'), size)


def text(draw, xy, message, size=24, color=INK, bold=False):
    draw.text(xy, message, font=font(size,bold), fill=color)


def compose(rgb, record, frame, index, both):
    im = Image.new('RGB', (1920,1080), '#0b1118')
    d = ImageDraw.Draw(im)
    text(d,(64,32),'WHITEBOARD BOT  /  PHYSICS VALIDATION',22,GREEN,True)
    text(d,(1450,34),'MUJOCO  /  SIMULATION ONLY',21,GREEN,True)
    text(d,(64,87),record['config']['title'],48,INK,True)
    text(d,(64,153),'Free 6-DOF chassis  ·  3 driven hubs  ·  36 passive rollers  ·  force-driven contact',25,MUTED)
    d.rounded_rectangle((64,215,1232,945),radius=16,fill='#121e2a',outline='#26394b',width=2)
    d.rounded_rectangle((1260,215,1856,945),radius=16,fill='#121e2a',outline='#26394b',width=2)
    im.paste(Image.fromarray(rgb),(72,245))
    if record['config']['incline']:
        d.rounded_rectangle((94,270,720,315),radius=8,fill='#121e2a')
        text(d,(110,280),'Board coordinates: gravity acts down the board (-Y).',22,ORANGE)
    text(d,(94,221),'3D PHYSICS REPLAY  /  MODEL PI + ESO',18,GREEN,True)
    text(d,(94,906),'Camera follows the robot. Motion and contact come from the physics engine.',23,MUTED)
    status = frame['status'].replace('_',' ')
    text(d,(1300,250),'SUPERVISOR STATE',19,MUTED,True)
    text(d,(1300,286),status,30,ORANGE if frame['fault'] or frame['slip'] else GREEN,True)
    text(d,(1300,345),f"t = {frame['t']:.2f} s   |   PWM {frame['pwm'][0]:+d} / {frame['pwm'][1]:+d} / {frame['pwm'][2]:+d}",23)
    text(d,(1300,409),'Mean position error over 3-6 s',25,INK,True)
    text(d,(1300,453),'Model PI',24,BLUE)
    text(d,(1650,446),f"{both['model']['metrics']['post_event_mean_error_m']*1000:.1f} mm",32,BLUE,True)
    text(d,(1300,502),'Model PI + ESO',24,GREEN)
    text(d,(1650,495),f"{both['eso']['metrics']['post_event_mean_error_m']*1000:.1f} mm",32,GREEN,True)
    text(d,(1300,550),'Full-run metric; original task reference.',20,MUTED)
    text(d,(1300,594),f"Mass {record['config']['mass']:.2f} kg  |  Incline {record['config']['incline']:.0f} deg",23)
    text(d,(1300,631),f"Friction {frame['mu']:.2f}  |  Supply scale {frame['supply']:.2f}",23)
    text(d,(1300,668),f"Normal force {frame['normal_N']:.2f} N  |  Contacts {frame['contacts']}",23)
    # Actual board-coordinate displacement, not the derated controller reference.
    axis=1 if record['config']['incline'] else 0
    text(d,(1300,722),'BODY HEIGHT / m' if axis else 'HORIZONTAL POSITION / m',18,MUTED,True)
    vals=[f['pose'][axis] for r in both.values() for f in r['trace']]
    lo,hi=min(-.02,min(vals)),max(.05,max(vals));span=max(.1,hi-lo)
    left,top,w,h=1340,760,470,120
    for j in range(3):
        yy=top+j*h/2
        d.line((left,yy,left+w,yy),fill='#304455')
        text(d,(1294,yy-8),f'{hi-j*span/2:.2f}',15,MUTED)
    for variant,col in [('model',BLUE),('eso',GREEN)]:
        points=[(left+f['t']/8*w,top+(hi-f['pose'][axis])/span*h) for f in both[variant]['trace'][:index+1]]
        if len(points)>1:
            d.line(points,fill=col,width=3)
    text(d,(left,890),'0 s',16,MUTED)
    text(d,(left+w-25,890),'8 s',16,MUTED)
    text(d,(64,977),'Measured: moving-range motor speed dynamics. Assumed: mass, torque, roller geometry, friction, magnetic force and sensors.',23,MUTED)
    text(d,(64,1026),'Approximate digital model. No claim of validated wall adhesion, hardware deployment or physical fall prevention.',22,'#73899d')
    return im


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario',choices=list(SCENARIOS))
    ap.add_argument('--stills',action='store_true')
    args=ap.parse_args()
    records=json.loads((ROOT/'reports/mujoco-traces.json').read_text())
    OUT.mkdir(exist_ok=True)
    names=[args.scenario] if args.scenario else list(SCENARIOS)
    metadata=[]
    for name in names:
        both={r['variant']:r for r in records if r['scenario']==name}
        record=both['eso'];cfg=record['config'];trace=record['trace']
        p=Plant(mass=cfg['mass'],incline=cfg['incline'])
        renderer=mujoco.Renderer(p.m,height=648,width=1152)
        cam=mujoco.MjvCamera();cam.distance=.25;cam.azimuth=115;cam.elevation=-52
        writer=None
        if not args.stills:
            writer=imageio_ffmpeg.write_frames(str(OUT/f'mujoco-{name}.mp4'),(1920,1080),fps=20,quality=8,
                                               macro_block_size=8,codec='libx264',output_params=['-movflags','+faststart'])
            writer.send(None)
        try:
            # One second introductory freeze, 8 s physics, one second final freeze.
            for i in range(200) if writer else [min(90,len(trace)-1)]:
                index=min(len(trace)-1,max(0,i-20)) if writer else i
                f=trace[index]
                p.d.qpos[:]=f['qpos'];mujoco.mj_forward(p.m,p.d)
                cam.lookat[:]=p.d.xpos[p.body];cam.lookat[2]=.008
                renderer.update_scene(p.d,camera=cam)
                im=compose(renderer.render(),record,f,index,both)
                if writer:
                    writer.send(np.asarray(im))
                if (writer and i==110) or not writer:
                    im.save(OUT/f'mujoco-{name}.png')
        finally:
            if writer:
                writer.close()
            renderer.close()
        metadata.append({'scenario':name,'title':cfg['title'],'seconds':10,'fps':20,'resolution':[1920,1080]})
        print('Rendered MuJoCo',name,flush=True)
    (OUT/'physics-capture-manifest.json').write_text(json.dumps({'engine':mujoco.__version__,
        'method':'Native MuJoCo render of recorded states; 20 Hz playback, no smoothing. English overlay. Silent.',
        'simulation_only':True,'clips':metadata},indent=2)+'\n')


if __name__=='__main__':
    main()
