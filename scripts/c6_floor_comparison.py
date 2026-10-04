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
FILES=[('前进','W','02-forward','01-forward',2,1),
       ('后退','S','03-backward','02-backward',3,2),
       ('左移','A','04-left','03-left',4,3),
       ('右移','D','05-right','04-right',5,4)]

def latest_section():
    return heading_section()+'<details><summary>展开此前的抖动诊断与失败记录</summary>'+smoothness_section()+history_section()+'</details>'


def heading_section():
    sections=[]
    for folder,title,chart in [
        ('newboard-floor-heading-02','协调减速：同固件开关对比','coordinated-stop'),
        ('newboard-floor-heading-03','四方向与侧移重复','coordinated-repeat'),
        ('newboard-floor-heading-04','方向反馈路径：同固件对比','direct-heading')]:
        p=ROOT/'reports'/folder/'heading-results.json'
        if not p.exists():continue
        r=json.loads(p.read_text());rows=[]
        for t in r['trials']:
            label=Path(t['source']).name
            rows.append(f'<tr><th>{escape(label)}</th><td>{t["encoder_peak_heading_deg"]:.2f}°</td>'
                f'<td>{t["encoder_final_heading_deg"]:+.2f}°</td><td>{"通过" if t["motion_quality_passed"] else "未通过"}</td></tr>')
        conclusion=''
        if 'initial_promotion_criteria_passed' in r:
            conclusion='<p>'+('首轮比较通过，仍需重复和车身观察。' if r['initial_promotion_criteria_passed'] else '未满足预定采用判据；保留数据，不把采集成功写成改善。')+'</p>'
        sections.append(f'<h3>{title}</h3><p><a href="{chart}/heading-comparison.png">英文曲线：打开原图</a> · '
            f'<a href="../../reports/{folder}/heading-results.json">指标和原始数据哈希</a></p>'
            '<div class="scroll"><table><thead><tr><th>记录</th><th>全过程最大偏转</th><th>停稳后偏转</th><th>轮速筛选</th></tr></thead><tbody>'+''.join(rows)+'</tbody></table></div>'+conclusion)
    model_summary=''
    model_path=ROOT/'reports/newboard-floor-excitation-validation-02/model-validation.json'
    if model_path.exists():
        model=json.loads(model_path.read_text());passed=sum(t['passed'] for t in model['results'])
        model_summary=f'''<h3>带载系统辨识：训练和独立验证分开</h3>
<p>训练已采集三台电机两个方向，共 6 段。独立验证已尝试 {model['validation_attempt_count']} 段，完整采集 {model['validation_completed_count']} 段；达到预定模型标准的有 {passed} 个。未完成的记录不算通过，也不当作完成的失败测试。</p>
<p>首段验证在固定输出前中止：M2 平均输出约 131/255，速度约为目标的 85%，超过该实验的固定输出准入范围。用户已确认线松、没有障碍或触碰；原因尚不能仅靠编码器确定。没有为通过实验放宽功率上限。</p>
<p><a href="loaded-model/">英文模型曲线：选择电机和方向</a> · <a href="../../reports/newboard-floor-excitation-validation-02/model-validation.json">独立验证结果</a> · <a href="../../reports/newboard-floor-excitation-02/frozen-models.json">验证前冻结的模型</a></p>
<p>12 段尝试共保留 1847 个去重的控制采样（包含中止段的起步）。同方向六次起步中，M1 负向首次持续轮动出现于 124–265 ms，附近的 PWM 为 78–149/255；M2 正向为 135–249 ms、84–150/255。这里是编码器检测到轮动时的输出，不是实测启动转矩，也不能排除车体带动轮子的影响。</p>
<a href="loaded-model/startup-variation.png"><img src="loaded-model/startup-variation.png" alt="Real floor startup variation across six repeated launches in each motor direction"></a>
<p>这些拟合没有写入 PID 或前馈。固定 PWM 不等于固定力矩，短数据也不足以完整辨识车体耦合、供电变化和接触摩擦。</p>'''
    return '''<h2>最新实测：起步与停车扭头</h2>
<p class="notice"><b>车身平顺性仍未通过。</b>用户最新观察：稍有改善，但起步和最后仍会转一下。不能用停稳后的误差小，替代整个过程平顺。</p>
<p>已实现移动中的编码器方向修正和约 200 ms 的正常减速目标；未在 550 ms 内减速到静止会停机。急停、断联和故障仍立即清零输出。新实验分别比较方向修正经过速度平滑和直接进入轮速环的效果，电机输出变化限制保持不变。</p>
<p>协调减速首轮：左移停稳偏转从 3.40° 降到 0.99°，右移从 2.90° 降到 0.60°。后续六段候选复测的最终偏转均不超过 0.66°，但过程中最大仍达 2.80°。</p>
<p>当前部署与开关状态见 <a href="../../reports/newboard-floor-heading-current.json">实际停止状态记录</a>。这些功能仍为 USB 开发试验，没有写入全速度或墙面保持验证标志；原无线页的驾驶锁尚未解除。</p>
<p><b>当前已恢复 rev14。</b>rev15 的左移瞬时偏转超出采用标准，已撤回。更快减力的候选未通过独立仿真筛查，没有烧录；全部失败记录保留。</p>
<nav><a href="http://192.168.4.1/">原小车控制页</a><a href="coordinated-stop/heading-comparison.png">协调减速英文对比图</a><a href="direct-heading/heading-comparison.png">最新反馈路径英文对比图</a></nav>
<a href="coordinated-stop/heading-comparison.png"><img src="coordinated-stop/heading-comparison.png" alt="Real floor encoder heading including start and stop, before and after coordinated deceleration"></a>
<p>每段约 0.08 m/s、1 秒。所有角度来自轮子编码器，不是相机或 IMU 测得的车身角度。没有称重或电流、电压测量；这些数据不能证明无打滑或墙面不下滑。</p>
'''+model_summary+''.join(sections)


def smoothness_section():
    return '''<h2>当前重点：消除实际抖动，尚未通过</h2>
<p class="notice"><b>历史 rev10：连续 PI，无单独启动脉冲。</b>用户在木地板上反馈“差不多，仍然抖”。后续 rev11、rev12 均未改善到可接受程度，已回退并确认停止。没有把轮速均值通过写成整车平顺。</p>
<div class="scroll"><table><thead><tr><th>候选</th><th>实际结果</th><th>状态</th></tr></thead><tbody>
<tr><th>rev10 连续 PI</th><td>右移末秒平均轮速约 100%，但 M1 起步峰值 177%；用户仍感到抖动。</td><td>历史基线，未通过平顺性验收</td></tr>
<tr><th>rev11 运行后降低增益</th><td>M1、M3 明显掉速；M3 末秒标准差达到目标的 75%。</td><td>否决并回退</td></tr>
<tr><th>rev12 编码器边沿时间测速</th><td>M1、M3 峰值约 203% / 200%；M3 末秒标准差 20.006%，超过预设 20%。用户反馈仍抖。</td><td>否决并回退</td></tr>
</tbody></table></div>
<p>固定输出诊断：此前约 0.7 秒固定输出段，用户觉得稍好，仍要求更好。随后单轮 ±4/255 扰动时，另一轮输出固定为 80/255 却接近停转；因此不能把短时均值当作已辨识的稳态模型。尚不能区分全部接触阻力、电源和机械因素。</p>
<h3>速度区间对比：0.08 m/s 的左右短测试</h3>
<p>两段均使用当时的 rev10，持续约 1 秒，理论行程同为 8 cm。右移三轮峰值约 104% / 119% / 104%；末段 400 ms 的标准差约为目标的 7.6% / 3.2% / 7.1%。左移三轮峰值约 101% / 99% / 124%，末段标准差约 3.4% / 1.6% / 3.4%。相比此前 0.04 m/s 右移，轮速曲线更好，但各一段不能证明所有速度、地面或负载都平顺。</p>
<p><a href="speed-range/floor-response.html">0.08 m/s 英文曲线与全部方向记录</a> · <a href="../../reports/newboard-floor-speed-01/manifest.json">实验范围与采集计划</a></p>
<p class="notice">四方向首轮 4/4、换序复测 4/4 通过短时轮速筛选；这不等于整车平顺。复测起步差仍达 140 ms，单段编码器估计方向变化最大 4.7°，起步同步与直线偏移尚未解决。<a href="speed-repeat/floor-response.html">复测全部曲线</a> · <a href="../../reports/newboard-floor-speed-validation-01/comparison-metrics.json">复测指标与原始哈希</a></p>
<nav><a href="continuous-pi/floor-response.html">rev10 英文曲线</a><a href="scheduled-pi/floor-response.html">rev11 失败曲线</a><a href="edge-speed/floor-response.html">rev12 与启动前中止记录</a><a href="excitation/floor-excitation.png">最新带载输入扰动：英文图</a></nav>
<a href="excitation/floor-excitation.png"><img src="excitation/floor-excitation.png" alt="Real floor independent-input experiment: fixed motor 3 duty with falling speed; model not identified"></a>
<p>每个候选都保留原始记录、固件哈希和停止验证。此处的标准差、峰值都是编码器轮速指标，不是车体振动测量。小车没有独立位置反馈，墙面保持、完整速度范围和无线驾驶尚未取得资格。</p>
<nav><a href="../../reports/newboard-floor-response-09/scored-results.json">rev12 分数与源文件哈希</a><a href="../../reports/newboard-floor-response-09/rollback-verified.json">回退及停止验证</a><a href="../../reports/newboard-floor-excitation-01/01-left-m1.json">输入扰动原始记录</a></nav>
'''


def history_section():
    sections=[]
    for folder,title,charts in [
        ('newboard-floor-response-06','rev9 首轮：4 个方向','start-release'),
        ('newboard-floor-validation-03','独立重复：两个顺序、8 段','start-release-repeat')]:
        path=ROOT/'reports'/folder/'scored-results.json'
        if not path.exists():continue
        result=json.loads(path.read_text());rows=[]
        for t in result['results']:
            values=' / '.join(f'M{w["motor"]} {w["mean_percent"]:.1f}%' for w in t['last_second'])
            rows.append(f'<tr><th>{escape(t["source"])}</th><td>{values}</td><td>{max(w["std_percent"] for w in t["last_second"]):.1f}%</td><td>{"通过" if t["passed"] else "未通过"}</td></tr>')
        sections.append(f'<h3>{title}：{sum(t["passed"] for t in result["results"])}/{len(result["results"])} 通过</h3>'
            f'<p><a href="{charts}/floor-response.html">全部英文曲线，可选方向并放大</a> · <a href="../../reports/{folder}/scored-results.json">指标、判据结果与哈希</a></p>'
            '<div class="scroll"><table><thead><tr><th>记录</th><th>最后一秒平均轮速 / 目标</th><th>最大轮速标准差 / 目标</th><th>结果</th></tr></thead><tbody>'+''.join(rows)+'</tbody></table></div>')
    return '''<h2>历史 rev9：轮速通过，整车仍抖动</h2>
<p>此前的 <b>robust-wheels-9-start-release</b> 检测到起转后及时降低启动推力，避免减力斜坡继续施加额外推力。以下为该历史版本的完整结果，当前版本见本页顶部。</p>
<p class="notice"><b>实际乘载表现未通过：</b>用户观察为“抖动很难受，我要丝滑”。以下 12/12 只表示轮速筛选通过，不能作为整车平顺性通过。正在以起步冲击、PWM 来回变化和三轮同步为目标继续修改。</p>
<p class="notice">0.04 m/s、每段 2 秒（理论行程 8 cm）的首轮 4/4 与独立重复 8/8 均通过：最后一秒平均轮速误差 ≤10%、标准差 ≤20%，末尾 200 ms 平均误差 ≤20%，同时要求正常起转、停止确认和连续采样。它们是低速轮速结果；未验证高速、独立车体直线精度或墙面保持，无线全速度控制仍未解锁。</p>
<p>独立重复中最后一秒最大平均误差 5.64%，最大标准差 12.55%。侧向启动仍存在差异：编码器估计的单段方向变化最大约 3.7°。因此不能把这些通过结果写成“整车已经完美走直线”。</p>
<a href="start-release-repeat/floor-response.html"><img src="start-release-repeat/floor-response-01.png" alt="Latest independent rightward floor trial, English chart"></a>
'''+''.join(sections)+'''
<h3>失败与扰动记录均保留</h3><ul>
<li><a href="loaded-start/floor-response.html">前一版探索测试</a>：长达 2 秒的四方向筛选通过；<a href="loaded-start-repeat/floor-response.html">独立重复</a>中电机 3 最后一秒偏高 13.2%，未通过更严格的重复判据。</li>
<li><a href="pi-only-contact/floor-response.html">手碰到的 PI-only 记录</a>：用户报告触碰，时间点未知，单独标注并保留原始文件，不计入正常行走表现。</li>
<li><a href="pi-only-repeat/floor-response.html">手离开后的重测</a>：电机 3 末尾轮速 77.8%，仍未通过。该次失败没有被删除或改成通过。</li>
</ul><h2>早期 0.6 秒短测试：历史基线与候选 1</h2>'''

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
        rows.append(f'<tr><th>{name} {key}</th><td>{values(a)}</td><td>{values(b)}</td><td>{"通过本页判据" if b["motion_quality_passed"] else "未通过"}</td></tr>')
        manifest.append(dict(direction=key,baseline=a,candidate=b))
    out=ROOT/'portfolio/newboard-floor'
    out.mkdir(exist_ok=True)
    (ROOT/'reports/newboard-floor-response-02/comparison.json').write_text(json.dumps(dict(
        condition='Real floor tests; 0.04 m/s requests lasting approximately 0.6 s',
        repeated_validation=False,body_position_measured=False,qualified_for_wireless=False,trials=manifest),indent=2))
    options=''.join(f'<option value="{key}">{name} {key}</option>' for name,key,*_ in FILES)
    images={key:[f'floor-response-{bi:02d}.png',f'loaded-pi-1/floor-response-{ai:02d}.png'] for _,key,_,_,bi,ai in FILES}
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>WhiteBoardBot · 新板地面对比</title>
<style>body{font:18px/1.65 system-ui;margin:30px auto;padding:0 20px;max-width:1150px;color:#183245;background:#f7fafb}h1{font-size:30px}a{color:#086e84}nav{display:flex;flex-wrap:wrap;gap:16px}.notice{padding:16px;background:#fff0dc;border-left:4px solid #bd7522}table{width:100%;border-collapse:collapse;background:white;font-size:16px}th,td{border-bottom:1px solid #dce5ea;padding:10px;text-align:left}.scroll{overflow:auto}select{font:inherit;padding:8px}img{width:100%;height:auto;display:block}.pair{display:grid;grid-template-columns:1fr;gap:24px}figure{margin:0}figcaption{font-weight:650}small{color:#536674}</style>
<h1>新板地面实测：最新结果与完整记录</h1>
<nav><a href="../hardware-response.html">全部实测</a><a href="floor-response.html">基线与时间戳中止</a><a href="loaded-pi-1/floor-response.html">候选 1 全部曲线</a><a href="loaded-pi-2/floor-response.html">候选 2 停机记录</a></nav>
__LATEST__
<p>每段目标 0.04 m/s，持续约 0.6 秒（理论行程 2.4 cm），随后自动停止。表中百分比为停止前约 200 ms 的编码器平均轮速 ÷ 请求轮速；100% 为目标，0% 表示几乎没转。</p>
<p class="notice">基线四方向 0/4 通过；候选 1 为 2/4：前进、左移改善，后退超调、右移仍掉速。候选 2 的右移触发电机 3 无反馈保护，未完成采集。各方向只有一次完整对比，不代表重复验证、车体走直线或墙面保持已通过；无线驾驶尚未解锁。</p>
<div class="scroll"><table><thead><tr><th>方向</th><th>基线末段轮速</th><th>候选 1 末段轮速</th><th>候选 1 结果</th></tr></thead><tbody>__ROWS__</tbody></table></div>
<p>查看方向 <select id="direction">__OPTIONS__</select>。点击图片可看原尺寸，英文图表可下载用于 portfolio。</p>
<div class="pair"><figure><figcaption>基线</figcaption><a id="before-full" target="_blank"><img id="before" alt="Baseline real floor wheel speed and PWM"></a></figure><figure><figcaption>候选 1：带载 PI</figcaption><a id="after-full" target="_blank"><img id="after" alt="Candidate real floor wheel speed and PWM"></a></figure></div>
<p>电机 3 正向 180/255、0.12 秒脉冲和后来单独的反向 180/255、0.06 秒脉冲都产生了正确极性的编码器反馈；两次停止后的滑行都超过对应试验的编码器位移或转角上限。这支持“两个方向都有反馈”，但两次完整试验仍不合格。</p>
<nav><a href="../../reports/newboard-floor-response-02/comparison.json">对比数据与源文件哈希</a><a href="../../reports/newboard-floor-response-03/m3-breakaway-check.json">短脉冲及滑行记录</a></nav>
<p><small>没有称重、电机电压、电流或独立车体位置测量。所有原始数据和失败记录均保留；轮速接近目标不等于没有打滑。</small></p>
<script>const images=__IMAGES__;function show(){const pair=images[document.getElementById('direction').value];['before','after'].forEach((id,i)=>{document.getElementById(id).src=pair[i];document.getElementById(id+'-full').href=pair[i]})}document.getElementById('direction').onchange=show;show();</script></html>'''
    (out/'index.html').write_text(page.replace('__LATEST__',latest_section()).replace('__ROWS__',''.join(rows)).replace('__OPTIONS__',options).replace('__IMAGES__',json.dumps(images)))
    print(out/'index.html')

if __name__=='__main__':main()
