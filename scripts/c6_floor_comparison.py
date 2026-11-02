#!/usr/bin/env python3
"""Publish the first new-board floor comparison from retained real captures.

The first pre-fix telemetry abort remains linked. Each direction has exactly
one completed baseline and one completed first-candidate trial; no repeats
are pooled or selected by score. Does not access hardware.
"""
import json
from html import escape
from pathlib import Path
from c6_floor_response_report import summarize

ROOT=Path(__file__).resolve().parents[1]/'c6_drive'
FILES=[('Forward','W','02-forward','01-forward',2,1),
       ('Backward','S','03-backward','02-backward',3,2),
       ('Left','A','04-left','03-left',4,3),
       ('Right','D','05-right','04-right',5,4)]

def latest_section():
    return heading_section()+'<details><summary>Show earlier jitter diagnostics and failure records</summary>'+smoothness_section()+history_section()+'</details>'


def heading_section():
    sections=[]
    for folder,title,chart in [
        ('newboard-floor-heading-02','Coordinated deceleration: on/off comparison on the same firmware','coordinated-stop'),
        ('newboard-floor-heading-03','Four directions and sideways repeats','coordinated-repeat'),
        ('newboard-floor-heading-04','Heading feedback path: same-firmware comparison','direct-heading')]:
        p=ROOT/'reports'/folder/'heading-results.json'
        if not p.exists():continue
        r=json.loads(p.read_text());rows=[]
        for t in r['trials']:
            label=Path(t['source']).name
            rows.append(f'<tr><th>{escape(label)}</th><td>{t["encoder_peak_heading_deg"]:.2f}°</td>'
                f'<td>{t["encoder_final_heading_deg"]:+.2f}°</td><td>{"Passed" if t["motion_quality_passed"] else "Failed"}</td></tr>')
        conclusion=''
        if 'initial_promotion_criteria_passed' in r:
            conclusion='<p>'+('First-round comparison passed; repeats and body observation are still needed.' if r['initial_promotion_criteria_passed'] else 'Did not meet the preset adoption criteria; data kept, and a successful capture is not reported as an improvement.')+'</p>'
        sections.append(f'<h3>{title}</h3><p><a href="{chart}/heading-comparison.png">English chart: open the original image</a> · '
            f'<a href="../../reports/{folder}/heading-results.json">Metrics and raw-data hashes</a></p>'
            '<div class="scroll"><table><thead><tr><th>Record</th><th>Max heading change (whole run)</th><th>Heading change after settling</th><th>Wheel-speed screening</th></tr></thead><tbody>'+''.join(rows)+'</tbody></table></div>'+conclusion)
    model_summary=''
    model_path=ROOT/'reports/newboard-floor-excitation-validation-02/model-validation.json'
    if model_path.exists():
        model=json.loads(model_path.read_text());passed=sum(t['passed'] for t in model['results'])
        model_summary=f'''<h3>Loaded system identification: training and independent validation kept separate</h3>
<p>Training captured all three motors in both directions, 6 runs in total. Independent validation has attempted {model['validation_attempt_count']} runs, with {model['validation_completed_count']} fully captured; {passed} met the preset model standard. Incomplete records do not count as passes, nor as completed failed tests.</p>
<p>The first validation run aborted before the fixed-output phase: M2 average output was about 131/255 and speed about 85% of target, outside this experiment's fixed-output entry range. The user confirmed the cable was slack, with no obstacle or contact; the cause cannot yet be determined from the encoders alone. The power limit was not relaxed to pass the experiment.</p>
<p><a href="loaded-model/">English model charts: choose motor and direction</a> · <a href="../../reports/newboard-floor-excitation-validation-02/model-validation.json">Independent validation results</a> · <a href="../../reports/newboard-floor-excitation-02/frozen-models.json">Models frozen before validation</a></p>
<p>The 12 attempted runs keep 1847 deduplicated control samples in total (including the start of the aborted run). Across six starts in the same direction, M1 negative first sustained wheel motion appeared at 124–265 ms, with PWM nearby at 78–149/255; M2 positive was 135–249 ms, 84–150/255. This is the output when the encoder detected wheel motion, not a measured starting torque, and the effect of the body dragging the wheel cannot be ruled out.</p>
<a href="loaded-model/startup-variation.png"><img src="loaded-model/startup-variation.png" alt="Real floor startup variation across six repeated launches in each motor direction"></a>
<p>These fits were not written into the PID or feedforward. Fixed PWM is not fixed torque, and short data is not enough to fully identify body coupling, supply variation and contact friction.</p>'''
    return '''<h2>Latest real test: heading twist at start and stop</h2>
<p class="notice"><b>Body smoothness still has not passed.</b> User's latest observation: slightly better, but it still turns a little at the start and at the end. A small error after settling does not replace smoothness over the whole run.</p>
<p>Encoder heading correction while moving and a normal deceleration target of about 200 ms are implemented; failing to slow to a standstill within 550 ms triggers a shutdown. E-stop, disconnect and faults still zero the output immediately. The new experiment compares heading correction routed through speed smoothing with heading correction fed directly into the wheel-speed loop; the motor output rate limit is unchanged.</p>
<p>Coordinated deceleration, first round: the settled heading change dropped from 3.40° to 0.99° for Left and from 2.90° to 0.60° for Right. In the six follow-up candidate retests the final heading change was at most 0.66°, but the maximum during the run still reached 2.80°.</p>
<p>For the current deployment and switch states, see the <a href="../../reports/newboard-floor-heading-current.json">actual stop-state record</a>. These features are still USB development trials; no full-speed or wall-hold validation flag was written, and the drive lock on the original wireless page is still on.</p>
<p><b>rev14 is restored for now.</b> rev15's transient heading change in Left exceeded the adoption standard, so it was withdrawn. A candidate that reduces force faster failed independent simulation screening and was not flashed; all failure records are kept.</p>
<nav><a href="http://192.168.4.1/">Original robot control page</a><a href="coordinated-stop/heading-comparison.png">Coordinated deceleration comparison chart (English)</a><a href="direct-heading/heading-comparison.png">Latest feedback path comparison chart (English)</a></nav>
<a href="coordinated-stop/heading-comparison.png"><img src="coordinated-stop/heading-comparison.png" alt="Real floor encoder heading including start and stop, before and after coordinated deceleration"></a>
<p>Each run is about 0.08 m/s for 1 second. All angles come from the wheel encoders, not body angles measured by a camera or IMU. There was no weighing and no current or voltage measurement; this data cannot prove there is no slip or that the robot will not slide down the wall.</p>
'''+model_summary+''.join(sections)


def smoothness_section():
    return '''<h2>Current focus: remove real jitter, not passed yet</h2>
<p class="notice"><b>Past rev10: continuous PI, no separate start pulse.</b> On a wood floor the user reported "about the same, still jittery". The later rev11 and rev12 did not improve it to an acceptable level; both were rolled back and the stop was confirmed. A passing mean wheel speed is not reported as a smooth robot.</p>
<div class="scroll"><table><thead><tr><th>Candidate</th><th>Actual result</th><th>Status</th></tr></thead><tbody>
<tr><th>rev10 continuous PI</th><td>Right: last-second mean wheel speed about 100%, but M1 start peak 177%; the user still felt jitter.</td><td>Past baseline; failed smoothness acceptance</td></tr>
<tr><th>rev11 lower gain once running</th><td>M1 and M3 clearly lost speed; M3 last-second standard deviation reached 75% of target.</td><td>Rejected and rolled back</td></tr>
<tr><th>rev12 speed from encoder edge timing</th><td>M1 and M3 peaks about 203% / 200%; M3 last-second standard deviation 20.006%, above the preset 20%. The user reported it was still jittery.</td><td>Rejected and rolled back</td></tr>
</tbody></table></div>
<p>Fixed-output diagnosis: with the earlier fixed-output phase of about 0.7 seconds, the user found it slightly better but still wanted it better. Later, during a ±4/255 disturbance on one wheel, another wheel with its output fixed at 80/255 came close to stalling; so a short-term mean cannot be treated as an identified steady-state model. Contact resistance, power and mechanical factors cannot all be told apart yet.</p>
<h3>Speed range comparison: short Left/Right tests at 0.08 m/s</h3>
<p>Both runs used the rev10 of that time, lasted about 1 second, and had the same theoretical travel of 8 cm. Right: three-wheel peaks about 104% / 119% / 104%; standard deviation over the final 400 ms about 7.6% / 3.2% / 7.1% of target. Left: three-wheel peaks about 101% / 99% / 124%, final standard deviation about 3.4% / 1.6% / 3.4%. Compared with the earlier 0.04 m/s Right test, the wheel-speed curves are better, but one run each cannot prove smoothness at all speeds, floors or loads.</p>
<p><a href="speed-range/floor-response.html">0.08 m/s English charts and all direction records</a> · <a href="../../reports/newboard-floor-speed-01/manifest.json">Experiment scope and capture plan</a></p>
<p class="notice">Four directions: first round 4/4 and reordered retest 4/4 passed the short wheel-speed screening; this does not mean the robot is smooth. In the retest the start difference still reached 140 ms, and the largest encoder-estimated heading change in one run was 4.7°; start synchronization and straight-line drift are not solved yet. <a href="speed-repeat/floor-response.html">All retest charts</a> · <a href="../../reports/newboard-floor-speed-validation-01/comparison-metrics.json">Retest metrics and raw hashes</a></p>
<nav><a href="continuous-pi/floor-response.html">rev10 English charts</a><a href="scheduled-pi/floor-response.html">rev11 failure charts</a><a href="edge-speed/floor-response.html">rev12 and pre-start abort records</a><a href="excitation/floor-excitation.png">Latest loaded input disturbance: English chart</a></nav>
<a href="excitation/floor-excitation.png"><img src="excitation/floor-excitation.png" alt="Real floor independent-input experiment: fixed motor 3 duty with falling speed; model not identified"></a>
<p>Every candidate keeps its raw records, firmware hash and stop verification. The standard deviations and peaks here are encoder wheel-speed metrics, not body vibration measurements. The robot has no independent position feedback; wall hold, the full speed range and wireless driving are not qualified yet.</p>
<nav><a href="../../reports/newboard-floor-response-09/scored-results.json">rev12 scores and source file hashes</a><a href="../../reports/newboard-floor-response-09/rollback-verified.json">Rollback and stop verification</a><a href="../../reports/newboard-floor-excitation-01/01-left-m1.json">Input disturbance raw records</a></nav>
'''


def history_section():
    sections=[]
    for folder,title,charts in [
        ('newboard-floor-response-06','rev9 first round: 4 directions','start-release'),
        ('newboard-floor-validation-03','Independent repeat: two orders, 8 runs','start-release-repeat')]:
        path=ROOT/'reports'/folder/'scored-results.json'
        if not path.exists():continue
        result=json.loads(path.read_text());rows=[]
        for t in result['results']:
            values=' / '.join(f'M{w["motor"]} {w["mean_percent"]:.1f}%' for w in t['last_second'])
            rows.append(f'<tr><th>{escape(t["source"])}</th><td>{values}</td><td>{max(w["std_percent"] for w in t["last_second"]):.1f}%</td><td>{"Passed" if t["passed"] else "Failed"}</td></tr>')
        sections.append(f'<h3>{title}: {sum(t["passed"] for t in result["results"])}/{len(result["results"])} passed</h3>'
            f'<p><a href="{charts}/floor-response.html">All English charts; choose a direction and zoom in</a> · <a href="../../reports/{folder}/scored-results.json">Metrics, criteria results and hashes</a></p>'
            '<div class="scroll"><table><thead><tr><th>Record</th><th>Last-second mean wheel speed / target</th><th>Max wheel-speed std. dev. / target</th><th>Result</th></tr></thead><tbody>'+''.join(rows)+'</tbody></table></div>')
    return '''<h2>Past rev9: wheel speed passed, robot still jittery</h2>
<p>The earlier <b>robust-wheels-9-start-release</b> lowers the start push promptly once it detects the wheel starting to turn, so the force-reduction ramp does not keep adding extra push. Below are the full results of that past version; the current version is at the top of this page.</p>
<p class="notice"><b>Actual ride performance failed:</b> the user's observation was "the jitter is really unpleasant, I want it silky smooth". The 12/12 below only means the wheel-speed screening passed; it does not count as the robot passing smoothness. Work continues, targeting the start jolt, PWM swinging back and forth, and three-wheel synchronization.</p>
<p class="notice">At 0.04 m/s, 2 seconds per run (theoretical travel 8 cm), both the first round 4/4 and the independent repeat 8/8 passed: last-second mean wheel-speed error ≤10%, standard deviation ≤20%, mean error over the final 200 ms ≤20%, with normal start-up, stop confirmation and continuous sampling also required. These are low-speed wheel-speed results; high speed, independent body straight-line accuracy and wall hold are not validated, and wireless full-speed control is still locked.</p>
<p>In the independent repeat, the largest last-second mean error was 5.64% and the largest standard deviation 12.55%. Sideways starts still differ: the largest encoder-estimated heading change in one run was about 3.7°. So these passes cannot be reported as "the robot already drives perfectly straight".</p>
<a href="start-release-repeat/floor-response.html"><img src="start-release-repeat/floor-response-01.png" alt="Latest independent rightward floor trial, English chart"></a>
'''+''.join(sections)+'''
<h3>Failure and disturbance records are all kept</h3><ul>
<li><a href="loaded-start/floor-response.html">Previous exploratory test</a>: the four-direction screening of up to 2 seconds passed; in the <a href="loaded-start-repeat/floor-response.html">independent repeat</a>, motor 3 ran 13.2% high in the last second and failed the stricter repeat criteria.</li>
<li><a href="pi-only-contact/floor-response.html">PI-only record with hand contact</a>: the user reported contact at an unknown time; it is marked separately with the raw file kept, and not counted as normal driving performance.</li>
<li><a href="pi-only-repeat/floor-response.html">Retest with hands off</a>: motor 3 final wheel speed 77.8%, still failed. This failure was not deleted or changed to a pass.</li>
</ul><h2>Early 0.6-second short tests: past baseline and candidate 1</h2>'''

def main():
    rows=[];manifest=[]
    for name,key,before,after,bidx,aidx in FILES:
        a=summarize(ROOT/'reports/newboard-floor-response-01'/f'{before}.json')
        b=summarize(ROOT/'reports/newboard-floor-response-02'/f'{after}.json')
        assert a['capture_complete'] and b['capture_complete']
        assert a['hardware_label']==b['hardware_label'] and a['speed_mps']==b['speed_mps']==.04
        assert a['key']==b['key']==key
        def values(trial):
            return ' / '.join(f'M{w["motor"]}: {w["tail_mean_target_percent"]:.0f}%' for w in trial['wheels'])
        rows.append(f'<tr><th>{name} {key}</th><td>{values(a)}</td><td>{values(b)}</td><td>{"Passed the criteria on this page" if b["motion_quality_passed"] else "Failed"}</td></tr>')
        manifest.append(dict(direction=key,baseline=a,candidate=b))
    out=ROOT/'portfolio/newboard-floor'
    out.mkdir(exist_ok=True)
    (ROOT/'reports/newboard-floor-response-02/comparison.json').write_text(json.dumps(dict(
        condition='Real floor tests; 0.04 m/s requests lasting approximately 0.6 s',
        repeated_validation=False,body_position_measured=False,qualified_for_wireless=False,trials=manifest),indent=2))
    options=''.join(f'<option value="{key}">{name} {key}</option>' for name,key,*_ in FILES)
    images={key:[f'floor-response-{bi:02d}.png',f'loaded-pi-1/floor-response-{ai:02d}.png'] for _,key,_,_,bi,ai in FILES}
    page='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>WhiteBoardBot · New board floor comparison</title>
<style>body{font:18px/1.65 system-ui;margin:30px auto;padding:0 20px;max-width:1150px;color:#183245;background:#f7fafb}h1{font-size:30px}a{color:#086e84}nav{display:flex;flex-wrap:wrap;gap:16px}.notice{padding:16px;background:#fff0dc;border-left:4px solid #bd7522}table{width:100%;border-collapse:collapse;background:white;font-size:16px}th,td{border-bottom:1px solid #dce5ea;padding:10px;text-align:left}.scroll{overflow:auto}select{font:inherit;padding:8px}img{width:100%;height:auto;display:block}.pair{display:grid;grid-template-columns:1fr;gap:24px}figure{margin:0}figcaption{font-weight:650}small{color:#536674}</style>
<h1>New board floor tests: latest results and full records</h1>
<nav><a href="../hardware-response.html">All real tests</a><a href="floor-response.html">Baseline and timestamp aborts</a><a href="loaded-pi-1/floor-response.html">Candidate 1 all charts</a><a href="loaded-pi-2/floor-response.html">Candidate 2 shutdown record</a></nav>
__LATEST__
<p>Each run targets 0.04 m/s for about 0.6 seconds (theoretical travel 2.4 cm), then stops automatically. Percentages in the table are the encoder mean wheel speed over about 200 ms before the stop ÷ the requested wheel speed; 100% is the target and 0% means it barely turned.</p>
<p class="notice">Baseline passed 0/4 directions; candidate 1 passed 2/4: Forward and Left improved, Backward overshot and Right still lost speed. Candidate 2's Right run triggered the motor 3 no-feedback protection and did not finish capturing. Each direction has only one complete comparison; this does not mean repeat validation, straight body driving or wall hold has passed. Wireless driving is still locked.</p>
<div class="scroll"><table><thead><tr><th>Direction</th><th>Baseline final wheel speed</th><th>Candidate 1 final wheel speed</th><th>Candidate 1 result</th></tr></thead><tbody>__ROWS__</tbody></table></div>
<p>View direction <select id="direction">__OPTIONS__</select>. Click an image to see it full size; the English charts can be downloaded for the portfolio.</p>
<div class="pair"><figure><figcaption>Baseline</figcaption><a id="before-full" target="_blank"><img id="before" alt="Baseline real floor wheel speed and PWM"></a></figure><figure><figcaption>Candidate 1: loaded PI</figcaption><a id="after-full" target="_blank"><img id="after" alt="Candidate real floor wheel speed and PWM"></a></figure></div>
<p>Motor 3's forward 180/255, 0.12-second pulse and the later separate reverse 180/255, 0.06-second pulse both produced encoder feedback of the correct polarity; the coasting after both stops exceeded the encoder displacement or rotation limit of the matching trial. This supports "there is feedback in both directions", but both complete trials still failed.</p>
<nav><a href="../../reports/newboard-floor-response-02/comparison.json">Comparison data and source file hashes</a><a href="../../reports/newboard-floor-response-03/m3-breakaway-check.json">Short pulse and coasting records</a></nav>
<p><small>No weighing, motor voltage, current or independent body position measurement. All raw data and failure records are kept; wheel speed near target does not mean there was no slip.</small></p>
<script>const images=__IMAGES__;function show(){const pair=images[document.getElementById('direction').value];['before','after'].forEach((id,i)=>{document.getElementById(id).src=pair[i];document.getElementById(id+'-full').href=pair[i]})}document.getElementById('direction').onchange=show;show();</script></html>'''
    (out/'index.html').write_text(page.replace('__LATEST__',latest_section()).replace('__ROWS__',''.join(rows)).replace('__OPTIONS__',options).replace('__IMAGES__',json.dumps(images)))
    print(out/'index.html')

if __name__=='__main__':main()
