#!/usr/bin/env python3
"""Offline analysis of archived floor captures, including failed trials.

No hardware access. Completion of data capture is deliberately separate from
motion quality. Thresholds below are declared diagnostic criteria, not a motor
model fit or chassis-position measurement.
"""
import argparse
import hashlib
from html import escape
import json
from pathlib import Path
import statistics
import os


def consecutive(rows, predicate, count=3):
    for i in range(len(rows)-count+1):
        group = rows[i:i+count]
        if all(predicate(r) for r in group) and all(b[0]-a[0] <= 15 for a,b in zip(group,group[1:])):
            return group[0][0]
    return None


def summarize(path):
    report = json.loads(path.read_text())
    annotation_path=path.with_suffix('.annotation.json')
    annotation=json.loads(annotation_path.read_text()) if annotation_path.exists() else None
    if annotation:
        for source in (path,path.with_suffix('.jsonl')):
            if annotation['sources'].get(source.name)!=hashlib.sha256(source.read_bytes()).hexdigest():
                raise ValueError('Trial annotation does not match retained source: '+str(source))
    clean=not annotation or annotation.get('clean_performance_evidence',True)
    if report.get('schema')=='wbb-loaded-capture-1' and 'command_ms' not in report:
        initial=report.get('initial_status',{})
        return dict(source=str(path),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            raw_sha256=hashlib.sha256(path.with_suffix('.jsonl').read_bytes()).hexdigest(),
            hardware_label=report.get('hardware_label','Unspecified hardware'),
            controller_revision=initial.get('controller_revision','Unknown'),session=initial.get('session'),
            key={'forward':'W','backward':'S','left':'A','right':'D'}[report['plan']['direction']],
            speed_mps=report['plan']['body_request_mps'],requested_duration_s=report['plan'].get('motion_seconds'),
            annotation=annotation,clean_performance_evidence=clean,motion_started=False,
            capture_complete=False,capture_error=report.get('error','No accepted command'),fault=None,
            sample_count=0,driven_samples=0,maximum_gap_ms=None,motion_quality_passed=False,
            start_ms=None,stop_ms=None,stop_settle_ms=None,wheels=[],samples=[])
    if report.get('schema')=='wbb-loaded-capture-1':
        stage=dict(accepted_ms=report['command_ms'],target_cps=report['plan']['wheel_targets_cps'],
                   key={'forward':'W','backward':'S','left':'A','right':'D'}[report['plan']['direction']])
        initial=report['initial_status'];final=report.get('final_status',{})
        speed=report['plan']['body_request_mps'];complete=report['capture_complete']
    else:
        stage = report['stages'][0]
        if len(report['stages']) != 1:
            raise ValueError('This summary requires one straight-motion step per trial')
        initial=report['initial'];final=report.get('final',{})
        speed=report['plan']['requested_speed_mps'];complete=report.get('passed',False)
    start = stage['accepted_ms']
    statuses = []
    for line in path.with_suffix('.jsonl').read_text().splitlines():
        frame = json.loads(line)['frame']
        if frame.get('type') == 'status' and frame['session'] == initial['session']:
            statuses.append(frame)
    samples = sorted({row[0]:row for f in statuses for row in f.get('control_samples',[])
                      if row[0] >= start}.values())
    active_start = next(r[0] for r in samples if any(abs(v)>0 for v in r[5:8]))
    stop = next((r[0] for r in samples if r[0]>active_start and all(v==0 for v in r[5:11])), None)
    motion_end=report.get('release_command_ms',stop)
    active = [r for r in samples if motion_end is None or r[0]<motion_end]
    if len(active)<10:
        raise ValueError('Insufficient driven data')
    max_gap = max(b[0]-a[0] for a,b in zip(samples,samples[1:]))
    wheels = []
    for motor,target in enumerate(stage['target_cps']):
        if target == 0:
            continue
        sign = 1 if target>0 else -1
        onset = consecutive(active, lambda r: sign*r[2+motor]>100)
        t90 = consecutive(active, lambda r: r[2+motor]/target>=.9)
        tail = [r for r in active if r[0] >= active[-1][0]-190]
        mean = statistics.mean(r[2+motor] for r in tail)
        near_zero = [r for r in active if r[0] >= start+300]
        longest = current = 0
        previous = None
        for r in near_zero:
            contiguous = previous is None or r[0]-previous<=15
            current = (current+10 if contiguous else 10) if abs(r[2+motor])<30 else 0
            longest = max(longest,current);previous = r[0]
        tail_error = abs(mean-target)/abs(target)
        wheels.append(dict(motor=motor+1,target_cps=target,
            onset_ms=None if onset is None else onset-start,
            first_sustained_t90_ms=None if t90 is None else t90-start,
            tail_mean_cps=mean,tail_mean_target_percent=mean/target*100,
            tail_relative_error=tail_error,tail_mean_pwm=statistics.mean(r[8+motor] for r in tail),
            peak_pwm=max(abs(r[8+motor]) for r in active),
            longest_low_speed_after_300ms=longest,
            diagnostic_motion_ok=onset is not None and longest<200 and tail_error<=.2))
    fault = final.get('last_fault')
    if fault and fault.get('run') != final.get('control_run'):
        fault = None
    quiet = consecutive([r for r in samples if stop is not None and r[0]>=stop],
                        lambda r: all(abs(v)<30 for v in r[2:5]),count=5)
    return dict(source=str(path),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        raw_sha256=hashlib.sha256(path.with_suffix('.jsonl').read_bytes()).hexdigest(),
        hardware_label=report.get('hardware_label','Previous board'),
        controller_revision=initial.get('controller_revision','Unspecified revision'),
        session=initial['session'],key=stage['key'],speed_mps=speed,
        requested_duration_s=report['plan'].get('motion_seconds'),
        annotation=annotation,clean_performance_evidence=clean,
        capture_complete=complete,capture_error=report.get('error'),fault=fault,
        sample_count=len(samples),driven_samples=len(active),maximum_gap_ms=max_gap,
        motion_quality_passed=bool(clean and complete and not fault and quiet is not None and max_gap<=40 and wheels and all(w['diagnostic_motion_ok'] for w in wheels)),
        start_ms=start,stop_ms=stop,motion_end_ms=motion_end,
        controlled_release=bool(report.get('release_command_ms')),
        stop_settle_ms=None if quiet is None else quiet-stop,
        wheels=wheels,samples=samples)


def draw(result,directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':13})
    colors=['#2674b6','#008a70','#cf652b']
    paths=[]
    for n,trial in enumerate(result['trials'],1):
        fig,axes=plt.subplots(2,1,figsize=(10,7.4),sharex=True,layout='constrained')
        if not trial['samples']:
            for ax in axes:ax.set_axis_off()
            fig.suptitle('Capture aborted before an accepted motion command',fontsize=16,weight='bold')
            axes[0].text(.05,.6,trial['capture_error'],wrap=True,transform=axes[0].transAxes,fontsize=12)
            axes[1].text(.05,.5,'No driven samples. Retained as an aborted attempt, not a wheel-performance result.',
                         wrap=True,transform=axes[1].transAxes)
            name=f'floor-response-{n:02d}.png';fig.savefig(directory/name,dpi=150);plt.close(fig);paths.append(name)
            continue
        rows=trial['samples'];t=[(r[0]-trial['start_ms'])/1000 for r in rows]
        for w in trial['wheels']:
            i=w['motor']-1;sign=1 if w['target_cps']>0 else -1
            axes[0].plot(t,[r[2+i]/w['target_cps']*100 for r in rows],color=colors[i],lw=2.5,label=f'Motor {i+1}')
            axes[1].plot(t,[sign*r[8+i] for r in rows],color=colors[i],lw=2.5)
        end=trial.get('motion_end_ms',trial['stop_ms'])
        ts=(end-trial['start_ms'])/1000 if end is not None else t[-1]
        axes[0].plot([0,ts,ts,t[-1]],[100,100,0,0],color='#566475',ls='--',lw=1.2,label='Requested speed')
        for ax in axes:
            if trial['stop_ms'] is not None:
                ts=(trial['stop_ms']-trial['start_ms'])/1000
                ax.axvline(ts,color='#7e4650',ls=':',lw=2)
                ax.axvspan(ts,t[-1],color='#f3e6e6',alpha=.7)
            ax.grid(alpha=.18);ax.spines[['top','right']].set_visible(False)
        peak=max(r[2+w['motor']-1]/w['target_cps']*100 for r in rows for w in trial['wheels'])
        axes[0].set(ylabel='Wheel speed / target (%)',ylim=(-5,max(140,peak*1.1)))
        axes[0].legend(ncol=3,fontsize=11,loc='upper right')
        pwm=[(1 if w['target_cps']>0 else -1)*r[8+w['motor']-1] for r in rows for w in trial['wheels']]
        axes[1].set(ylabel='Applied PWM / 255',xlabel='Time after accepted command (s)',
                    ylim=(min(-5,min(pwm)-5),max(160,max(pwm)+10)))
        label='Firmware fault: '+trial['fault']['code'] if trial['fault'] else ('Capture aborted: '+str(trial['capture_error']) if not trial['capture_complete'] else ('Motion criteria PASSED' if trial['motion_quality_passed'] else 'Motion criteria FAILED'))
        if not trial['clean_performance_evidence']:label='User-reported hand contact; excluded from clean comparison'
        duration=f' / {trial["requested_duration_s"]:g} s' if trial['requested_duration_s'] else ''
        fig.suptitle(f'Floor response {trial["key"]} at {trial["speed_mps"]:.2f} m/s{duration}\n{label}',fontsize=16,weight='bold')
        fig.supxlabel(trial['hardware_label']+' | '+trial['controller_revision']+'\nReal 100 Hz wheel telemetry; no independent body position, voltage or current measurement.',fontsize=9)
        name=f'floor-response-{n:02d}.png';fig.savefig(directory/name,dpi=150);plt.close(fig);paths.append(name)
    return paths


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--directory',type=Path,default=Path('c6_drive/reports/floor-response-01'))
    ap.add_argument('--portfolio',type=Path,default=Path('c6_drive/portfolio'))
    args=ap.parse_args()
    trials=[summarize(p) for p in sorted(args.directory.glob('[0-9]*.json')) if not p.name.endswith('.annotation.json')]
    if not trials:ap.error('No floor capture files found')
    labels=sorted({t['hardware_label'] for t in trials})
    if len(labels)!=1:ap.error('Keep different hardware in separate result directories')
    args.portfolio.mkdir(parents=True,exist_ok=True)
    result=dict(condition='Floor captures: '+labels[0],
        criteria=dict(onset_cps=100,onset_consecutive_samples=3,tail_window_ms=200,
                      tail_error_limit_percent=20,low_speed_cps=30,low_speed_duration_limit_ms=200),
        steady_state_proven=False,model_fitted=False,body_position_measured=False,
        capture_count=len(trials),complete_captures=sum(t['capture_complete'] for t in trials),
        motion_quality_passes=sum(t['motion_quality_passed'] for t in trials),
        disturbed_captures=sum(not t['clean_performance_evidence'] for t in trials),
        unique_control_samples=sum(t['sample_count'] for t in trials),trials=trials)
    (args.directory/'analysis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    figures=draw(result,args.portfolio)
    direction={'W':'Forward','S':'Backward','A':'Left','D':'Right'}
    names=[f'{direction[t["key"]]} / {t["key"]} at {t["speed_mps"]:.2f} m/s'+
           (f' / {t["requested_duration_s"]:g} s' if t['requested_duration_s'] else '')+
           (' — hand contact' if not t['clean_performance_evidence'] else ' — fault' if t['fault'] else '') for t in trials]
    options=''.join(f'<option value="{p}">{names[i]}</option>' for i,p in enumerate(figures))
    analysis_url=os.path.relpath(args.directory/'analysis.json',args.portfolio)
    archive_url=os.path.relpath(Path('c6_drive/portfolio/hardware-response.html'),args.portfolio)
    html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>WhiteBoardBot · 带载实测数据</title><style>body{font:18px/1.65 system-ui;margin:32px auto;padding:0 20px;max-width:1100px;color:#172b3c;background:#f7f9fb}h1{font-size:32px}select{font:inherit;padding:10px;max-width:100%}img{width:100%;height:auto}a{color:#186a96}.notice{padding:16px;background:#fff1de;border-left:4px solid #b96d16}nav{display:flex;gap:20px;flex-wrap:wrap}</style>
<h1>带载数据：采完不代表控制通过</h1><nav><a href="__ARCHIVE__">返回实测结果</a><a href="__ANALYSIS__">指标和原始采样</a></nav>
<p class="notice">__COUNTS__</p>
<p>上图纵轴 100% 是请求速度，下图是实际 PWM。阴影从输出归零开始。标题会区分固件故障、采集中止和运动判据结果；中止的记录也保留。短时测试不代表已完成稳态或直线精度验证。</p>
<label>查看测试 <select id="trial">'''+options+'''</select></label> · <a id="full" href="floor-response-01.png" target="_blank">原尺寸图片</a>
<img id="chart" src="floor-response-01.png" alt="真实轮速及PWM随时间变化，英文图表">
<p>统计定义：连续 3 个采样超过 100 counts/s 记为起转；末段 200 ms 平均轮速误差要求不超过 20%；指令后 300 ms 以后，轮速低于 30 counts/s 持续 200 ms 则判运动不合格。这些是诊断筛选线，不是稳定性证明。没有电压、电流或独立车体位置测量，无法仅凭这批数据区分全部机械、电源和摩擦因素。</p>
<script>document.getElementById('trial').onchange=e=>{document.getElementById('chart').src=e.target.value;document.getElementById('full').href=e.target.value;};</script></html>'''
    counts=(f'{labels[0]}：{len(trials)} 段、{result["unique_control_samples"]} 个去重后的 100 Hz 采样；'
            f'{result["complete_captures"]} 段完成采集，{result["motion_quality_passes"]} 段通过本页运动判据。'
            f'{result["disturbed_captures"]} 段已标注人为扰动，不作为正常行走对比。'
            '位移只来自编码器，不是独立车体测量。')
    html=html.replace('__ARCHIVE__',escape(archive_url,quote=True)).replace('__ANALYSIS__',escape(analysis_url,quote=True)).replace('__COUNTS__',escape(counts))
    (args.portfolio/'floor-response.html').write_text(html)
    print(json.dumps({k:v for k,v in result.items() if k!='trials'},indent=2))


if __name__=='__main__':
    main()
