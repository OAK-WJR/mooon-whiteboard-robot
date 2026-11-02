# Local Wi-Fi provisioning and wireless diagnostics

**Latest deployment, 2026-10-03:** replacement screen board `WBB-C6-XXXX` now
connects to **`Home Network`** at **http://192.168.1.x/**. The exact network name
contains capitals and a space; the initially supplied `homenetwork` returned
`NO_AP_FOUND`. The corrected name was confirmed from the Mac's saved network
names, without reading its passwords or changing its connection. The user's
provided network password was submitted over USB and stored on the robot;
it is absent from source, firmware constants and experiment logs.

Upload/write-hash verification passed for the USB provisioning firmware.
Twenty fresh USB status frames confirmed connected/stopped state, followed by
**30/30 authenticated LAN status reads** (67.9 ms median, 140.9 ms maximum RTT).
Gains matched the pre-provision state. This replacement board remains
**uncalibrated**; no controller-validation flags or motion commands were sent.
At the user's subsequent explicit request, **Home Network now opens without a
second browser login**. The deployed page matches the source and returns HTTP
200 without `WWW-Authenticate`; 30/30 status reads passed with no credentials
(30.2 ms median, 35.7 ms maximum). The existing Owner Hotspot exception is retained.
Other network profiles retain their prior authentication behavior. This does
not unlock unvalidated motor control.
[No-login verification](reports/home-network-no-login-verified.json).
[Connection evidence](reports/home-network-usb-provision-20261003.json).

**Previous board's deployment:** the robot and Mac were connected through the user's
password-protected **`Owner Hotspot`** personal hotspot. The robot is at
**http://172.20.10.x/**. At the user's explicit request, that exact personal
SSID now opens the control page without a second browser login. Firmware upload
and write-hash verification passed; unauthenticated HTTP GETs returned the exact
compiled control page and **30/30 live status responses**, with median round-trip
time **39.6 ms** and maximum **65.2 ms**. The robot remained disarmed with zero
motor outputs, retained all six saved gain sets and rejoined the saved hotspot.
This was a read-only network test; no motion command or loaded-control test ran.
[Deployment evidence](reports/iphone-no-login-deploy.json).

**Earlier campus check:** a user-saved school PEAP profile was present, but
the C6 reports `201 / NO_AP_FOUND` and has no station IP. Its explicit 2.4 GHz
scan found **5 access points, 0 matching that SSID** at the current location.
The Mac remained connected to the school network; its system network report showed
channel **149 (5 GHz, 40 MHz)**. These observations establish a local availability
problem, not that every campus AP is configured the same way. Account validity
and campus peer reachability have not been tested. Use a mutually reachable
2.4 GHz personal hotspot or an IT-provided compatible network for the next
wireless test. See [diagnostic deployment evidence](reports/school-wifi-diagnostics-deploy.json).

**Flashed on 2026-09-22 after the user confirmed the robot was off the wall, motor power disconnected, and USB connected.** USB boot verification passed: 199 status samples stayed disarmed with all PWM registers and six output pads low; the display reported initialized, no fault was reported, and all six saved legacy gain sets matched the pre-flash values. Live HTTP provisioning and school association are still untested. This change does not deploy the unqualified load-servo candidate or alter motor gains, output limits or validation gates. See [deployment evidence](reports/wifi-deploy-20260922.json).

## Existing control page

Connect to this robot's `WBB-C6-XXXX` hotspot and open `http://192.168.4.1/`. Use the new **Wi-Fi network** section in the original driving page. Choose personal password authentication or the school's PEAP / TTLS method. Enterprise configuration additionally requires username, optional outer identity, the officially specified authentication-server name and its official CA certificate in PEM format. The user enters the network's credentials and selects **Save and connect**. No school credentials have been read, received, guessed or submitted by the agent.

HTTP configuration is accepted only through the private robot hotspot, while disarmed, with fresh control telemetry and all wheels below 30 counts/s. Personal Wi-Fi can also be provisioned over local USB as described below. This keeps account/password submissions out of HTTP forms on a shared campus LAN. Profiles are saved as one versioned NVS blob. Usernames, passwords, outer identities and CA contents are not returned in status or saved in browser storage. Saved credential storage is ordinary NVS, not a claim of encrypted flash.

The ESP32 runs AP+STA. Successful association gives it a router-assigned address; the screen and page display the IP and `wbb-c6-XXXX.local` name. The existing HTTP control/status routes work on both interfaces. The computer and robot must be on a mutually reachable local network. mDNS is a convenience; use the displayed IP if the network suppresses multicast.

On the user's password-protected `Owner Hotspot` hotspot or `Home Network` network,
the page and control/status routes require no second browser login. The exemption requires an
active station connection, personal authentication, a nonempty Wi-Fi password
and one of those exact configured SSIDs. Other shared LANs retain HTTP Digest authentication:
username **wbb**, password **the robot hotspot password**. The robot's own private
hotspot also opens the page directly. The connected screen shows the IP and
`OPEN IP IN BROWSER`. The chosen personal hotspot's reachable clients can open
the control page; this does not create a cloud relay or port-forwarding rule.

The AP remains available if joining fails. Personal-network attempts end after 15 seconds; enterprise attempts have 30 seconds. Retries wait 30 seconds and occur only while disarmed. Enterprise retries stop after three failed attempts until settings are saved again or the device restarts; a successful association resets the counter. New arm requests are blocked during an apply/connect attempt. Losing a previously connected STA link stops active control, and rejoining never arms or restores commands. The existing 300 ms command lease still expires independently of networking. Radio/channel changes may briefly interrupt the AP; initial setup and retries run with motors disarmed.

## Supported networks and school restrictions

- 2.4 GHz personal WPA2-compatible networks and deliberately selected open networks.
- Open means no link-layer password; captive-portal login is **not** implemented.
- PEAP and EAP-TTLS with MSCHAPv2 or PAP are implemented through the installed ESP-IDF 5.5.4 EAP APIs, with explicit method selection, a required custom CA and server-name verification. PAP here is inside the verified TTLS tunnel. The runtime parses the CA before storing it, checks each EAP setup API before connecting, and clears SDK-owned credential pointers before replacing their backing profile. There is no skip-certificate-verification option.
- EAP-TLS client certificates, certificate enrollment, captive-portal login and other EAP types are not implemented. The school network tested was PEAP/MSCHAPv2; its guest network is a separate web-login network.
- Client isolation, VLAN separation, captive portals and blocked mDNS can prevent access even after association. Joining an SSID is not proof that the computer can reach the robot.
- A mutually reachable 2.4 GHz phone hotspot can use the same provisioning form. Its suitability has not been measured in this session.

Espressif documents [the C6's 2.4 GHz radio and simultaneous STA+AP support](https://documentation.espressif.com/esp32-c6_datasheet_en.html), [station connection, hostname ordering and reconnect APIs](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html), and [NVS byte storage](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html). The installed Arduino-ESP32 3.3.9 `HttpAdvancedAuth` example and `WebServer.cpp` were checked for Digest authentication. No asynchronous Wi-Fi callback accesses motor state; network policy runs in the main loop.

Enterprise behavior was checked against the [version-matched Espressif security guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32c6/api-guides/wifi-security.html), [EAP API reference](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32c6/api-reference/network/esp_wifi.html), and installed `esp_eap_client.h` / `STA.cpp`. Certificate-chain and server-name settings are configured, and the EAP time-check disable flag is explicitly false. This is not a claim that certificate expiration/time behavior or school-server interoperability has been physically verified; those checks remain part of deployment validation.

## Live connection diagnostics

The 2026-10-03 boot test exposed an additional recovery failure: the robot
retained its IP and answered ping after a station disconnect, while port 80
connection attempts timed out. Firmware now counts station disconnect/stop and
got-IP events in the existing locked event snapshot. The main loop closes and
recreates the HTTP sockets when that generation changes, outside the motor
control mutex; an active lease is stopped before recovery. Callback threads do
not operate the server. Status exposes `http_recovery_revision: 1` and
`http_restarts`. The next real boot again recorded a station disconnect, recovered
the listener twice, and passed the unauthenticated page/30-status check above.
This follows Espressif's [socket lifecycle requirements after disconnect and
IP acquisition](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32c6/api-guides/wifi.html#wi-fi-event-description).
It is a boot/reconnect verification, not a guarantee of future Wi-Fi latency.

The original page and USB status now report the last Wi-Fi driver failure code,
association state, attempt count and explicit scan results. The asynchronous
Wi-Fi callback copies only non-secret fields under a dedicated lock and never
accesses motor state. Local disconnect codes 8 and 36 do not overwrite the radio
failure that triggered an attempt timeout. Authentication failures are not
presented as proof of an incorrect password. A successful connection hides old
failure messages.

The USB-only `wifi_scan` command starts a bounded asynchronous 2.4 GHz scan. It
requires disarmed, stopped wheels, fresh control telemetry, no ongoing connection
attempt and no existing station connection. Scanning blocks arming and defers
retries until it completes. Status exposes the total AP count and match count,
strongest channel/RSSI/auth mode for the configured SSID; other SSIDs, BSSIDs and
account credentials are not returned. No scan happens automatically.

The scan firmware was flashed with write-hash verification. All 666 observed
post-flash status frames stayed disarmed with zero motor PWM/pad readbacks; all
six saved gain sets were retained. Only the `wifi_scan` diagnostic command was
sent. The implementation follows the installed Arduino-ESP32 3.3.9 `WiFiScan.cpp`,
`STA.cpp` and event example, alongside the official [event/threading
documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html)
and [IDF 5.5.4 driver documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32c6/api-guides/wifi.html).
The actual page JavaScript contract tests and full C6 build pass.
See [deployment and hardware scan evidence](reports/school-wifi-diagnostics-deploy.json).

## USB provisioning without switching the computer's Wi-Fi

Firmware with `usb_wifi_revision: 1` accepts the USB-only `wifi_config` command
for personal Wi-Fi. HTTP `/cmd` rejects this command. The existing AP form and
USB share input validation and stopped-only NVS saving; flash writes happen
outside the control mutex. Network apply/retry still uses the existing policy
and cannot re-arm the robot. Enterprise CA configuration remains in the AP form.

```sh
.venv/bin/python scripts/c6_wifi_provision.py --ssid 'Home Network' --report c6_drive/reports/new-wifi-check.json
```

The tool prompts for a hidden password (never a command-line argument), checks
stopped outputs and the firmware revision, saves the profile, then verifies
association/IP and authenticated read-only LAN access. A still-active connection
attempt is allowed to finish before changing the profile. Reports contain
connection diagnostics but no network password or device login key. The device
key is obtained through the existing USB `info` operation and held only in
memory for the LAN check. A connection failure remains a failure in the report;
the saved profile is retained for diagnosis. This tool never arms or drives.

## Computer access without a USB tether

`scripts/c6_wifi_client.py` uses the existing web protocol. Construction and
`--status` only read status; they do not stop, reset or arm a robot. On the verified
iPhone hotspot, no key file is needed:

```sh
.venv/bin/python scripts/c6_wifi_client.py --url http://172.20.10.x/ --status
```

The existing short loaded-capture script now supports `--url` instead of `--port`. Physical execution still requires actual current authorization, an off-wall setup on a clear level floor, motor power and its `--execute --floor-confirmed` flags. Its `--plan` path is entirely offline. Wi-Fi captures use the same web calibration locks; raw PWM, gain writes and validation overrides remain USB-only. Authenticated trace toggling is allowed only while disarmed, for the existing 100 Hz sample ring. Gaps greater than 40 ms make capture analysis fail instead of inventing missing data.

Robot URLs cannot include credentials, queries, fragments or arbitrary paths. The client ignores system proxies, refuses redirects, rejects changed firmware sessions, and never prints the device key or control token. This transport has offline contract tests; physical latency and the browser-to-motor path are not validated yet.

## Validation completed

- Full `pio run -d c6_drive` build, including network, web, screen and motor runtime.
- Native network policy tests with address/undefined-behavior sanitizers: personal/enterprise profile requirements, malformed domains, still/disarmed requirements, timed apply/retry, enterprise retry exhaustion, link loss and clock rollover.
- Actual page JavaScript tests: personal/PEAP/TTLS submissions, refusing incomplete trust settings, no password in browser storage, no automatic arm, active-control locks, safe URL/text rendering, existing WASD/stop/calibration behavior.
- Isolated desktop/mobile browser rendering with fake network responses. Screenshots are mock UI evidence, not a connection to the robot.
- Five offline Wi-Fi transport tests and seven loaded-capture tests; these tests used mocks, not the robot.
- Authorized USB deployment: ESP32-C6FH8 revision 0.2 / 8 MB flash identified, full flash backed up with owner-only permissions, actual old/new data partitions compared before writing. PlatformIO upload and esptool write-hash verification passed.
- Live USB observation: first status reported `BOOT`, all 199 observed samples were ready/disarmed, control age was at most 9 ms, no fault was reported, and every motor PWM/duty/pad readback stayed zero. The LCD initialization flag was true; screen legibility was not visually checked. The network reported `ap_only`, no saved station profile, and `http://192.168.4.1/` on `WBB-C6-XXXX`. No arm or movement command was sent.

The firmware's existing build-specific saved controller validation did not carry across the new image. Live status confirmed calibrated legacy mode with unchanged saved gains, while robust/hold validation flags were false. Those modes require their established validation procedure before web activation. This network feature does not silently mark a control algorithm as validated.

## Firmware partition layout

The enterprise/CA verification build exceeds the old 1.25 MiB application slot. `partitions.csv` uses the existing 8 MiB flash with a 2.5 MiB app0 and a second application slot in upper flash. Before this upload, the actual device partition table was parsed from the complete 8 MB backup and compared: NVS (`0x9000`, `0x5000`), SPIFFS (`0x290000`, `0x160000`) and coredump (`0x3F0000`, `0x10000`) offsets/sizes are unchanged. The upload wrote the bootloader, partition table, OTA selection data and app0; it did not erase the whole chip or those three preserved data partitions. The private backup is `reports/wifi-deploy-20260922-backup.bin` (Git-ignored, mode 0600); image copies and SHA-256 hashes are recorded alongside the deployment report.
