#!/bin/bash
# 手动刷新 compile_commands.json 并清洗（供 clangd 使用）
# 用法: ./.scripts/refresh_clangd.sh

set -e
cd "$(dirname "$0")/.."
# 脚本位于 .scripts/ 目录下

echo ">> 生成 compile_commands.json ..."

if [ -x ".venv/bin/pio" ]; then
    .venv/bin/pio run -t compiledb
else
    pio run -t compiledb
fi

echo ""
echo ">> 清洗 compile_commands.json ..."

# 用 Python 清洗
PYTHON=".venv/bin/python3"
[ ! -x "$PYTHON" ] && PYTHON="python3"

$PYTHON -c "
import json, re, sys

REMOVE_FLAGS = {
    '-mlongcalls', '-mtext-section-literals', '-fno-tree-switch-conversion',
    '-fno-jump-tables', '-fstrict-volatile-bitfields', '-Wno-frame-address',
    '-freestanding',
}

with open('compile_commands.json', 'r') as f:
    entries = json.load(f)

for entry in entries:
    cmd = entry.get('command', '')
    if not cmd:
        continue
    cmd = re.sub(r'\S*xtensa-esp32[s23]*-elf-g\+\+', 'clang++', cmd)
    cmd = re.sub(r'\S*xtensa-esp32[s23]*-elf-gcc', 'clang', cmd)
    parts = [p for p in cmd.split() if p not in REMOVE_FLAGS and not p.startswith('-mfix-esp32')]
    entry['command'] = ' '.join(parts)

with open('compile_commands.json', 'w') as f:
    json.dump(entries, f, indent=4)

print('>> 清洗完成')
"

echo ""
echo ">> 完成! 请在 Cursor 中执行 Cmd+Shift+P → 'clangd: Restart language server'"
