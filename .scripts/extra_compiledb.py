"""
PlatformIO extra_script (pre):
- Enable COMPILATIONDB_INCLUDE_TOOLCHAIN so compile_commands.json contains full toolchain paths
- Write compile_commands.json to .pio/build/<env>/

Usage:
  pio run                  # normal build
  pio run -t compiledb     # generate compile_commands.json (for clangd)

Reference: https://docs.platformio.org/en/latest/integration/compile_commands.html
Community-confirmed solution: https://github.com/clangd/clangd/issues/2256
"""
import os

Import("env")

# Fix: Disable macOS Keyring to prevent pip from hanging in non-interactive environments
os.environ.setdefault("PYTHON_KEYRING_BACKEND", "keyring.backends.null.Keyring")
os.environ.setdefault("PIP_NO_INPUT", "1")

# compile_commands.json contains full toolchain paths (clangd needs them)
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

# Write to the build directory (.clangd CompilationDatabase points here)
env.Replace(COMPILATIONDB_PATH=os.path.join("$BUILD_DIR", "compile_commands.json"))
