#!/usr/bin/env python3
"""Build and flash c6_wall over Wi-Fi, then confirm the robot came back.

    python tools/ota.py               # build + upload + verify
    python tools/ota.py --no-build    # upload the last build
    python tools/ota.py --host <robot-ip>

The robot refuses an update while it is moving. It keeps holding position
during the upload, brakes through the ~1 s reboot and resumes holding. If the
new image cannot come back online it rolls back by itself within 150 s.
"""
import argparse
import http.client
import json
import os
import subprocess
import sys
import time
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = os.path.join(ROOT, ".pio", "build", "c6_wall", "firmware.bin")
HOST_CACHE = os.path.join(ROOT, ".robot_host")
PIO = os.path.join(os.path.dirname(ROOT), ".venv", "bin", "pio")
DEFAULT_HOSTS = ["192.168.4.1"]  # the robot's own hotspot; .robot_host caches the LAN address


def status(host, timeout=2.5):
    with urllib.request.urlopen(f"http://{host}/api/status", timeout=timeout) as r:
        return json.loads(r.read())


def find_robot(preferred):
    hosts = [preferred] if preferred else []
    if os.path.exists(HOST_CACHE):
        hosts.append(open(HOST_CACHE).read().strip())
    hosts += DEFAULT_HOSTS
    for host in dict.fromkeys(h for h in hosts if h):
        try:
            return host, status(host)
        except Exception:
            continue
    sys.exit("找不到小车：确认它和电脑在同一个 Wi-Fi，或用 --host 指定 IP")


def upload(host, data, token):
    conn = http.client.HTTPConnection(host, 80, timeout=60)
    conn.putrequest("POST", "/ota")
    conn.putheader("Content-Type", "application/octet-stream")
    conn.putheader("Content-Length", str(len(data)))
    conn.putheader("X-OTA-Token", token)
    conn.endheaders()
    step = 16384
    for off in range(0, len(data), step):
        conn.send(data[off:off + step])
        pct = min(100, (off + step) * 100 // len(data))
        print(f"\r  上传 {pct:3d}%", end="", flush=True)
    print()
    resp = conn.getresponse()
    body = resp.read().decode(errors="replace")
    try:
        return resp.status, json.loads(body)
    except ValueError:
        return resp.status, {"ok": False, "error": body}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--bin", help="upload this image instead (implies --no-build)")
    args = ap.parse_args()
    t0 = time.time()
    if not args.no_build and not args.bin:
        print("编译中…")
        log = os.path.join(ROOT, ".pio", "ota-build.log")
        os.makedirs(os.path.dirname(log), exist_ok=True)
        with open(log, "w") as f:
            if subprocess.run([PIO, "run"], cwd=ROOT, stdout=f, stderr=subprocess.STDOUT).returncode:
                sys.exit(f"编译失败，见 {log}")
        print(f"  编译完成 {time.time() - t0:.0f} s")
    data = open(args.bin or BIN, "rb").read()
    token = open(os.path.join(ROOT, ".ota_token")).read().strip()

    # esp_app_desc_t starts at byte 32; app_elf_sha256 is at byte 176. The fw id ends with its first 4 bytes.
    host, st = find_robot(args.host)
    old_fw = st["fw"]
    print(f"小车 {host}: 固件 {old_fw}, 状态 {st['mode']}/{st['src']}")
    same = old_fw.endswith("-" + data[176:180].hex())
    if same:
        print("  注意：这和小车上的是同一个固件，重新刷一遍")
    if st.get("moving"):
        sys.exit("小车正在运动，先停下再更新")

    t1 = time.time()
    code, reply = upload(host, data, token)
    if code != 200 or not reply.get("ok"):
        sys.exit(f"更新失败: HTTP {code} {reply}")
    print(f"  已写入 {len(data) / 1024:.0f} KB，用时 {time.time() - t1:.1f} s，重启中…")

    sent = time.time()
    deadline = sent + 90
    new = None
    while time.time() < deadline:
        time.sleep(0.7)
        try:
            new = status(host, timeout=1.5)
        except Exception:
            continue
        if (new["fw"] != old_fw or same) and 4000 < new["up"] < (time.time() - sent) * 1000:
            break
        # Old firmware with a fresh uptime: the new image crashed and the bootloader rolled back.
        if not same and new["fw"] == old_fw and new["up"] < (time.time() - sent) * 1000:
            sys.exit(f"新固件启动失败，已自动回滚到 {old_fw}（小车一直保持/刹车）")
    if not new or (new["fw"] == old_fw and not same):
        sys.exit("小车没有带着新固件回来（会在 150 s 内自动回滚到旧版）")
    for _ in range(10):  # the status request itself confirms the image
        if not new.get("ota_pending"):
            break
        time.sleep(0.5)
        new = status(host)
    ip = new["net"]["ip"] or host
    open(HOST_CACHE, "w").write(ip)
    print(f"完成: 固件 {new['fw']}，状态 {new['mode']}{'（重启后已恢复保持）' if new.get('resumed') else ''}，"
          f"{'已确认' if not new.get('ota_pending') else '尚未确认'}，总用时 {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
