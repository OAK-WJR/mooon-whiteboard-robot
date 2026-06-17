"""
PlatformIO extra_script (pre):
- 开启 COMPILATIONDB_INCLUDE_TOOLCHAIN，让 compile_commands.json 包含完整工具链路径
- 将 compile_commands.json 输出到 .pio/build/<env>/

用法:
  pio run                  # 正常编译
  pio run -t compiledb     # 生成 compile_commands.json (给 clangd 用)

参考: https://docs.platformio.org/en/latest/integration/compile_commands.html
社区确认方案: https://github.com/clangd/clangd/issues/2256
"""
import os

Import("env")

# Fix: Disable macOS Keyring to prevent pip from hanging in non-interactive environments
os.environ.setdefault("PYTHON_KEYRING_BACKEND", "keyring.backends.null.Keyring")
os.environ.setdefault("PIP_NO_INPUT", "1")

# compile_commands.json 包含完整工具链路径 (clangd 需要)
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

# 输出到 build 目录 (.clangd CompilationDatabase 指向这里)
env.Replace(COMPILATIONDB_PATH=os.path.join("$BUILD_DIR", "compile_commands.json"))
