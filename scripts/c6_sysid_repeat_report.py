#!/usr/bin/env python3
"""Audit independent captures against a frozen PWM-to-speed model; never opens USB."""
import argparse
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path

import numpy as np

from c6_drive_sysid import metrics, series_high_rate, simulate, stopped, write_json


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_capture(data):
    if (data.get('status') != 'COLLECTED_NOT_TUNED' or not data.get('stopped')
            or data.get('condition') != 'suspended' or not data.get('hardware_id')):
        raise ValueError('Need a complete, identified, stopped suspended capture')
    expected = {(m, s, r) for m in (1, 2, 3) for s in (1, -1)
                for r in ('train', 'validation')}
    runs = data['runs']
    if len(runs) != 12 or {(r['motor'], r['sign'], r['role']) for r in runs} != expected:
        raise ValueError('Need exactly one complete set of twelve runs')
    if not all(r.get('complete') for r in runs):
        raise ValueError('Incomplete run')
    final = data.get('final')
    if final and (not final.get('ready') or not stopped(final)):
        raise ValueError('Final hardware stop not verified')


def audit(model_data, captures):
    """First capture supplies training runs; all later captures remain held out."""
    models = {(m['motor'], m['sign']): m for m in model_data['models']}
    if len(models) != 6 or model_data.get('condition') != 'suspended':
        raise ValueError('Need six frozen suspended-wheel models')
    board = captures[0]['hardware_id']
    seen_runs = set()
    seen_driven = set()
    results, traces, counts = [], [], []
    for repeat, data in enumerate(captures, 1):
        validate_capture(data)
        if data['hardware_id'] != board:
            raise ValueError('Do not mix different boards')
        all_rows, driven_rows = set(), set()
        for run in data['runs']:
            key = (run['session'], run['lease'])
            if key in seen_runs:
                raise ValueError('Repeated experiment is not independent validation')
            seen_runs.add(key)
            t, u, y, quality = series_high_rate(run)
            motor = run['motor'] - 1
            for frame in run['frames']:
                for row in frame['control_samples']:
                    sample_key = (run['session'], row[0])
                    all_rows.add(sample_key)
                    if row[8 + motor] != 0:
                        driven_rows.add(sample_key)
            if repeat == 1 and run['role'] == 'train':
                continue
            model = models[(run['motor'], run['sign'])]
            prediction = simulate(t, u, y[0], model)
            error = metrics(y, prediction)
            passed = error['r_squared'] >= .9 and error['rmse_fraction_of_speed_span'] <= .1
            case_id = f'r{repeat}-m{run["motor"]}-{ "pos" if run["sign"] > 0 else "neg"}-{run["role"]}'
            result = {'id': case_id, 'repeat': repeat, 'motor': run['motor'], 'sign': run['sign'],
                      'profile': 'A' if run['role'] == 'train' else 'B',
                      'original_role': run['role'], 'lease': run['lease'],
                      'evaluated_samples': len(t), 'quality': quality, **error,
                      'passed': bool(passed)}
            results.append(result)
            traces.append((result, t, u, y, prediction))
        if seen_driven & driven_rows:
            raise ValueError('Captures reuse measured control samples')
        seen_driven |= driven_rows
        counts.append({'repeat': repeat, 'complete_runs': len(data['runs']),
                       'unique_control_samples': len(all_rows),
                       'driven_control_samples': len(driven_rows)})
    report = {'schema': 'wbb-sysid-repeat-audit-1', 'created_utc': datetime.now(timezone.utc).isoformat(),
              'hardware_id': board, 'condition': 'suspended', 'gains_written': False,
              'model_refitted_to_holdouts': False, 'captures': counts,
              'complete_runs': sum(c['complete_runs'] for c in counts),
              'driven_control_samples': len(seen_driven),
              'evaluated_samples': sum(r['evaluated_samples'] for r in results),
              'heldout_runs': len(results), 'passed_runs': sum(r['passed'] for r in results),
              'criteria': {'r_squared_min': .9, 'rmse_fraction_of_speed_span_max': .1},
              'results': results,
              'limitations': ['Suspended wheels only; 120–200 PWM, one motor at a time.',
                              'First second excluded: moving dynamics, not breakaway identification.',
                              'Model initialized from first validation speed only; no subsequent state correction.',
                              'Filtered encoder speed, not chassis velocity; no current or battery-voltage measurement.',
                              'No loaded, simultaneous-wheel, slip or wall-hold validation.',
                              'All holdouts retained, including failed prediction criteria.'],
              'sources': model_data.get('sources', [])}
    return report, traces


def render(report, traces, output_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    for result, t, u, y, predicted in traces:
        fig, axes = plt.subplots(2, 1, figsize=(12, 6.7), sharex=True,
                                 gridspec_kw={'height_ratios': [3, 1]}, layout='constrained')
        ax, command = axes
        ax.plot(t, y, color='#067a76', lw=1.9, label='Measured encoder speed')
        ax.plot(t, predicted, color='#d97128', lw=1.7, ls='--', label='Frozen first-repeat model')
        ax.set_ylabel('Speed magnitude (counts/s)', fontsize=12)
        ax.legend(loc='upper right', fontsize=11)
        ax.grid(alpha=.2)
        command.step(t, u, where='post', color='#334155', lw=1.6)
        command.set_ylim(100, 220)
        command.set_yticks([120, 160, 200])
        command.set_ylabel('PWM / 255', fontsize=12)
        command.set_xlabel('Time after excluded startup interval (s)', fontsize=12)
        command.grid(alpha=.2)
        direction = 'positive' if result['sign'] > 0 else 'negative'
        fig.suptitle(f'WhiteBoardBot | Motor {result["motor"]}, {direction} | Repeat {result["repeat"]}, profile {result["profile"]}\n'
                     f'Independent prediction: R² {result["r_squared"]:.3f} · RMSE {result["rmse_cps"]:.0f} counts/s · '
                     f'{"PASS" if result["passed"] else "BELOW CRITERION"}', fontsize=16)
        fig.supxlabel('Real suspended-wheel data · 100 Hz control trace · No loaded or wall-hold validation', fontsize=10)
        fig.savefig(output_dir / (result['id'] + '.png'), dpi=150)
        plt.close(fig)
    options = []
    table = []
    for r in report['results']:
        label = f'第{r["repeat"]}轮 · 电机{r["motor"]} · {"正向" if r["sign"] > 0 else "反向"} · 曲线{r["profile"]} · {"达标" if r["passed"] else "未达标"}'
        options.append(f'<option value="{r["id"]}.png">{html.escape(label)}</option>')
        table.append(f'<tr><td>{html.escape(label)}</td><td>{r["r_squared"]:.3f}</td>'
                     f'<td>{100*r["rmse_fraction_of_speed_span"]:.1f}%</td></tr>')
    worst = min(report['results'], key=lambda r: r['r_squared'])['id'] + '.png'
    options = [o.replace(f'value="{worst}"', f'value="{worst}" selected') for o in options]
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>新屏幕板：三轮实测与独立验证</title><style>
body{font:18px/1.6 system-ui,sans-serif;max-width:1200px;margin:28px auto;padding:0 20px;background:#f8fafc;color:#182a3a}
h1{font-size:30px}select{font:inherit;padding:10px;max-width:100%}img{width:100%;height:auto;background:white;margin-top:18px}
a{color:#086960}table{border-collapse:collapse;width:100%}th,td{text-align:left;padding:8px;border-bottom:1px solid #d7e0e5}
.metric{font-size:24px;font-weight:700}small{color:#526372}</style>
<h1>新屏幕板：三轮实测与独立验证</h1><p><a href="../hardware-response.html">返回原响应页面</a> · <a href="audit.json">完整指标与来源</a></p>
'''
    page += (f'<p class="metric">{report["complete_runs"]} 段完整采集 · {report["driven_control_samples"]:,} 个驱动中采样点</p>'
             f'<p>三轮悬空，逐轮正反转，PWM 120–200/255。仅用第一轮的 6 段 A 曲线拟合，参数随后冻结。'
             f'余下 {report["heldout_runs"]} 段独立预测中，{report["passed_runs"]} 段达到预先设定的精度，'
             f'{report["heldout_runs"]-report["passed_runs"]} 段未达标。采集完成不等于控制器已通过。</p>'
             '<p>绿实线是实测轮速，橙虚线是模型预测，下方是实际 PWM。两线越接近越好；此处不是目标跟踪 PID 测试。'
             '判据：R² ≥ 0.90，且 RMSE / 实测速度范围 ≤ 10%。默认显示误差最大的一段。</p>'
             '<label for="case">选择一张大图：</label> <select id="case">' + ''.join(options) + '</select>'
             f'<p><a id="full" href="{worst}" target="_blank">打开完整分辨率英文图片</a></p>'
             f'<img id="chart" src="{worst}" alt="实测编码器速度与冻结模型预测，以及 PWM 输入">'
             '<p>碰触中断的段落另存并排除。以上只覆盖已经转动的悬空轮子，排除了每段开头 1 秒；'
             '不代表起步死区、带载直行、打滑或墙面保持已解决。没有写入新 PID 或解除固件验证限制。</p>'
             '<p>本次第三轮电机 1 反向 A 曲线还有一次短暂速度突降；PWM 当时保持不变，原因尚未确定。'
             '该段保留在评分中，不能直接认定是人为碰触或电机损坏。'
             '<a href="../../reports/newboard-suspended-anomaly-01.json">查看原始计数与边沿记录</a>。</p>'
             '<details><summary>全部独立预测结果</summary><table><thead><tr><th>实验</th><th>R²</th><th>RMSE / 速度范围</th></tr></thead><tbody>'
             + ''.join(table) + '</tbody></table></details>'
             '<p><small>方法参考：<a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html">WPILib SysId diagnostics</a>。'
             '原始文件 SHA-256 与硬件身份见完整指标。</small></p>'
             '<script>document.getElementById("case").onchange=function(){document.getElementById("chart").src=this.value;document.getElementById("full").href=this.value;};</script></html>')
    (output_dir / 'index.html').write_text(page)
    write_json(output_dir / 'audit.json', report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--captures', nargs='+', required=True, help='Reference capture first, then untouched repeats')
    parser.add_argument('--report', required=True)
    parser.add_argument('--gallery', required=True)
    args = parser.parse_args()
    report, traces = audit(json.loads(Path(args.model).read_text()),
                           [json.loads(Path(p).read_text()) for p in args.captures])
    report['provenance'] = {'frozen_model': {'file': args.model, 'sha256': digest(args.model)},
                            'captures': [{'file': p, 'sha256': digest(p)} for p in args.captures]}
    write_json(args.report, report)
    render(report, traces, args.gallery)
    print(json.dumps({k: report[k] for k in ('complete_runs', 'driven_control_samples',
                                          'evaluated_samples', 'heldout_runs', 'passed_runs')}))


if __name__ == '__main__':
    main()
