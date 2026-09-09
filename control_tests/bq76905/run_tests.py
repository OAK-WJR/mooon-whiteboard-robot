#!/usr/bin/env python3
"""Offline standalone host tests. Reads only explicit public sources + owned files.

Never invokes PlatformIO, discovers devices, or reads project configuration/secrets.
All build output stays inside this test directory.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--compiler", default="clang++")
    parser.add_argument("--no-sanitizers", action="store_true",
                        help="For a compiler/host without ASan/UBSan; the report records this limitation")
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    root = here.parents[1]
    contract = json.loads((here / "source_contract.json").read_text())
    base = root / contract["retained_base"]
    source_results = []
    for name, expected in contract["retained_sha256"].items():
        # Exact allowlist from the retained contract. No recursive hashing/search.
        if name not in {"bq76905.pdf", "bq76905.txt", "bq76905_trm.pdf", "bq76905_trm.txt"}:
            raise RuntimeError("Unapproved source path")
        actual = hashlib.sha256((base / name).read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError("Retained source changed; review the source contract: " + name)
        source_results.append({"file": name, "sha256": actual, "match": True})
    text = (base / "bq76905_trm.txt").read_text()
    for check in contract["trm_text_checks"]:
        if not re.search(check["regex"], text, re.MULTILINE):
            raise RuntimeError("TI table check failed: " + check["fact"])
    print(f"PASS {len(source_results)} retained hashes, {len(contract['trm_text_checks'])} actual TI text checks", flush=True)

    build = here / ".build"
    build.mkdir(exist_ok=True)
    binary = build / "test_monitor"
    compiler = shutil.which(args.compiler)
    if not compiler:
        raise RuntimeError("Compiler unavailable: " + args.compiler)
    command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-g", "-O1"]
    if not args.no_sanitizers:
        command += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    # Only these two translation units. Include search does NOT open include/config.h.
    command += ["-I" + str(root / "include"), str(root / "src/bq76905_monitor.cpp"),
                str(here / "test_monitor.cpp"), "-o", str(binary)]
    subprocess.run(command, check=True, cwd=here)
    result = subprocess.run([str(binary)], cwd=here, text=True, capture_output=True)
    print(result.stdout, end="")
    print(result.stderr, end="", file=sys.stderr)
    report = {
        "generated_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "scope": "Host software/protocol model only; no physical BQ/USB/serial/charging validation",
        "compiler": subprocess.check_output([compiler, "--version"], text=True).splitlines()[0],
        "standard": "C++11",
        "sanitizers": [] if args.no_sanitizers else ["address", "undefined"],
        "sources": source_results,
        "source_table_checks": len(contract["trm_text_checks"]),
        "compile_command": command,
        "exit_code": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
        "owned_code_sha256": {
            str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [root / "include/bq76905_monitor.h", root / "src/bq76905_monitor.cpp",
                         here / "test_monitor.cpp", here / "device_model.h", here / "source_contract.json"]
        }
    }
    (here / "test_report.json").write_text(json.dumps(report, indent=2) + "\n")
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
