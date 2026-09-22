#!/usr/bin/env python3
"""Large, separate charts from archived hardware samples (no simulated data)."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parents[1] / 'c6_drive'
old = json.loads((root/'reports/robust-hardware-bench-01.json').read_text())
new = json.loads((root/'reports/robust-hardware-r4-full.json').read_text())
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 16, 'svg.fonttype': 'none'})
for name, title in [('same-speed', 'Equal wheel targets'), ('lateral', 'Sideways command')]:
    fig, axes = plt.subplots(2, 1, figsize=(9, 9), layout='constrained', sharex=True, sharey=True)
    for ax, report, controller, label in zip(axes, [old, new], ['legacy', 'robust_wheels'], ['BEFORE', 'AFTER']):
        r = next(r for r in report['results'] if r['name'] == name and r['controller'] == controller)
        samples = np.array(r['samples']); t = (samples[:, 0]-r['command_ms'])/1000
        for i, target in enumerate(r['values']):
            if target:
                ax.plot(t, samples[:, 2+i]/target*100, lw=3,
                        color=['#2674b6', '#008a70', '#cf652b'][i], label=f'Motor {i+1}')
        ax.axhline(100, color='#4f5b69', ls='--', lw=1.5)
        ax.axhspan(90, 110, color='#008a70', alpha=.08)
        ax.set(xlim=(0, 1), ylim=(0, 130), ylabel='Target speed (%)')
        ax.set_title(f'{label}  |  90% crossing gap: {r["t90_spread_s"]*1000:.0f} ms', loc='left', weight='bold')
        ax.grid(alpha=.15); ax.spines[['top', 'right']].set_visible(False)
    axes[0].legend(ncol=3, loc='lower right', fontsize=13)
    axes[1].set_xlabel('Seconds after command')
    fig.suptitle(title, fontsize=24, weight='bold')
    fig.supxlabel('Real encoders, wheels suspended. Dashed line = requested speed.\n' +
                  ('Sideways repeat range: 50–130 ms after revision.\n' if name == 'lateral' else '') +
                  'This measures wheel response, not chassis position or wall holding.', fontsize=12)
    fig.savefig(root/f'portfolio/hardware-{name}-large.png', dpi=140)
    fig.savefig(root/f'portfolio/hardware-{name}-large.svg')
    plt.close(fig)
