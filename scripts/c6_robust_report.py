#!/usr/bin/env python3
"""Summarize actual suspended-wheel logs; retain failed development trials."""
import hashlib
import json
from pathlib import Path
from statistics import mean, median
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]/'c6_drive'
REPORTS=ROOT/'reports'


def main():
    names=['robust-hardware-r4-full','robust-lateral-r4-repeat','robust-lateral-r4-reverse']
    reports=[json.loads((REPORTS/(n+'.json')).read_text()) for n in names]
    for report in reports:
        if report.get('error'):raise RuntimeError(report['error'])
        for path,digest in report['source_sha256'].items():
            assert hashlib.sha256((ROOT.parent/path).read_bytes()).hexdigest()==digest, 'Stale firmware report: '+path
    runs=[r for report in reports for r in report['results']]
    motors=[m for r in runs for m in r['motors']]
    first=json.loads((REPORTS/'robust-hardware-bench-01.json').read_text())
    previous={r['name']:r for r in first['results'] if r['controller']=='legacy'}
    current={r['name']:r for r in reports[0]['results']}
    repeated=json.loads((REPORTS/'robust-lateral-r2-repeat.json').read_text())
    transitions=json.loads((REPORTS/'robust-transitions-r4.json').read_text())
    ready_path=REPORTS/'robust-ready-r4.json'
    ready=json.loads(ready_path.read_text()) if ready_path.exists() else None
    assert transitions.get('passed'), 'Transition checks incomplete'
    groups=lambda rows:[r['t90_spread_s']*1000 for r in rows]
    old_spreads=groups([r for r in repeated['results'] if r['controller']=='legacy'])
    new_spreads=groups(reports[1]['results'])
    reverse_spreads=groups(reports[2]['results'])
    summary={
        'simulation_only':False,'suspended':True,'controller_revision':'robust-wheels-4',
        'steady_tracking_trials':len(runs),'steady_tracking_passes':sum(r['tracking_pass'] for r in runs),
        'max_steady_relative_error':max(m['relative_error'] for m in motors),
        'mean_steady_relative_error':mean(m['relative_error'] for m in motors),
        'max_overshoot_fraction':max(m['overshoot_fraction'] for m in motors),
        'minimum_quadrature_efficiency':min(m['quadrature_efficiency'] for m in motors),
        'max_trace_gap_ms':max(r['max_gap_ms'] for r in runs),
        'same_speed_t90_spread_ms':{'legacy':previous['same-speed']['t90_spread_s']*1000,'candidate':current['same-speed']['t90_spread_s']*1000},
        'lateral_t90_spread_ms':{'legacy_repetitions':old_spreads,'candidate_repetitions':new_spreads,'candidate_reverse':reverse_spreads},
        'command_timeout_output_zero_ms':next(r['zero_after_ms'] for r in transitions['results'] if r['phase']=='command_timeout'),
        'transition_checks':[r['phase'] for r in transitions['results']],
        'development_failure_retained':{'report':'robust-lateral-r2-repeat.json','error':repeated.get('error'),
            'finding':'M1 produced almost no encoder motion at 140 PWM while M2/M3 moved. A bounded 160 PWM open-loop check then produced valid motion in both directions. This supports inadequate breakaway at some stops, but does not exclude intermittent hardware faults.'},
        'limitations':['Suspended tests at 1500-3000 counts/s; no floor, wall, loaded, thermal or wireless end-to-end qualification.',
            'Capture resolution is 10 ms. Response times include target shaping and filtered encoders, not pure motor delay.',
            'Early controller comparisons are sequential and exploratory; motor temperature and supply voltage were not measured.',
            'Full simulated pose/slip/hold controller is not deployed; this firmware uses the wheel-only adapter.'],
        'bench_final_state':transitions['final'],
        'final_state':ready['status'] if ready else transitions['final'],
        'candidate_selected_for_current_session':bool(ready),
        'candidate_selection_persistent':False,
        'source_sha256':reports[0]['source_sha256'],
    }
    summary['dynamic_screen_pass']=all(r['tracking_pass'] for r in runs) and summary['max_overshoot_fraction']<=.20 and max(new_spreads+reverse_spreads)<=150
    (REPORTS/'robust-hardware-summary.json').write_text(json.dumps(summary,indent=2)+'\n')

    plt.rcParams.update({'font.family':'DejaVu Sans','svg.fonttype':'none','font.size':10})
    fig,axs=plt.subplots(2,2,figsize=(13,8),layout='constrained',sharex=True,sharey=True)
    fig.suptitle('WhiteBoardBot | Real hardware response, wheels suspended',fontsize=18,weight='bold')
    colors=['#377bb5','#14896e','#cc7048']
    for row,name in enumerate(['same-speed','lateral']):
        for col,(label,records) in enumerate([('Previous wheel PI',previous),('Revised model PI + observer',current)]):
            ax=axs[row,col];r=records[name];samples=np.array(r['samples']);t=(samples[:,0]-r['command_ms'])/1000
            for i,target in enumerate(r['values']):
                if target:ax.plot(t,100*samples[:,2+i]/target,color=colors[i],lw=1.7,label=f'Motor {i+1}')
            ax.axhspan(90,110,color='#769987',alpha=.1);ax.axhline(100,color='#738193',lw=.8,ls='--')
            ax.set_title(f"{label}\n{'Equal targets' if name=='same-speed' else 'Lateral wheel vector'} · T90 spread {r['t90_spread_s']*1000:.0f} ms")
            ax.set(xlim=(0,1),ylim=(0,145),xlabel='Time after accepted command / s',ylabel='Measured speed / requested speed (%)')
            ax.grid(alpha=.18);ax.spines[['top','right']].set_visible(False);ax.legend(loc='lower right',fontsize=9)
    caption=(f"Lateral repeats: legacy {min(old_spreads):.0f}–{max(old_spreads):.0f} ms (n={len(old_spreads)}); revised {min(new_spreads):.0f}–{max(new_spreads):.0f} ms (n={len(new_spreads)}). "
             f"10 ms samples; comparisons are exploratory.\nNo body-position feedback or wall-hold validation. Earlier M1 startup failure remains in the raw reports.")
    fig.supxlabel(caption,fontsize=10)
    fig.savefig(ROOT/'portfolio/hardware-response.png',dpi=170);fig.savefig(ROOT/'portfolio/hardware-response.svg');plt.close(fig)
    lines=['# Suspended hardware test results','',
        'The wheel-only adapter was flashed and tested on the actual C6 robot. This is encoder/PWM evidence, not a physical-motion video or wall-climbing qualification.','',
        f"Revision: **robust-wheels-4**. Completed {len(runs)} speed fixtures, plus direct reversal, zero-command release and command-timeout checks.",'',
        f"Maximum steady speed error: **{summary['max_steady_relative_error']:.2%}**. Maximum observed transient overshoot: **{summary['max_overshoot_fraction']:.2%}**. Minimum quadrature efficiency: **{summary['minimum_quadrature_efficiency']:.2%}**.",'',
        f"Same-speed T90 spread: {summary['same_speed_t90_spread_ms']['legacy']:.0f} → {summary['same_speed_t90_spread_ms']['candidate']:.0f} ms. Lateral repeated T90 spread: old {min(old_spreads):.0f}–{max(old_spreads):.0f} ms; revised {min(new_spreads):.0f}–{max(new_spreads):.0f} ms; opposite direction {min(reverse_spreads):.0f}–{max(reverse_spreads):.0f} ms.",'',
        f"Output went to zero {summary['command_timeout_output_zero_ms']} ms after the last command. Test tools restored legacy mode, disabled trace and verified stopped outputs at completion.",'',
        ('After reviewing the completed tests and matching source hashes, the candidate was selected and validated for this session. The robot remains disarmed, with three zero duties and LOW output pads. Reboot returns to legacy mode; no gains or controller choice were saved in NVS.' if ready else 'The candidate has not been selected for user driving.'),'',
        'The earlier 140-PWM startup revision had one `NO_ENCODER` stop on M1. No automatic re-arm occurred. At 160 PWM, a separate bounded pulse produced normal forward/reverse counts. M1 now uses at most 160 PWM for at most 250 ms at low-speed startup, handing back to the dynamic controller once filtered motion exceeds 150 counts/s. M2/M3 retain 140-PWM startup. The original failure is retained; mechanical or electrical intermittency has not been ruled out.','',
        'T90 is the first crossing of 90% requested speed, not proof of settling. The raw reports also retain 200 ms settling windows and transient overshoot. These short tests cover 1500–3000 counts/s and do not establish performance at the 10000-count/s command ceiling.','',
        '[English plot](../portfolio/hardware-response.png) · [JSON summary](robust-hardware-summary.json)','',
        '## Raw evidence','']
    for name in names+['robust-transitions-r4','robust-hardware-bench-01','robust-lateral-r2-repeat','robust-motor1-breakaway-01']:
        lines.append(f'- [{name}.json]({name}.json)')
    (REPORTS/'robust-hardware-summary.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ['final_state','source_sha256']},indent=2))


if __name__=='__main__':main()
