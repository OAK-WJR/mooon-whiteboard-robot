> **Archived (2026-09-21 → 2026-10-04).** This firmware was replaced by
> [`c6_wall`](../c6_wall) after the robot slid down the whiteboard: encoder-only
> control could not detect chassis slip. The raw evidence logs under `reports/`
> (about 1 GB of JSON) are not published, so links into `reports/` do not resolve.
> Plots and simulation captures are in [`portfolio/`](portfolio). Most notes below
> were written during the work and are kept as they were.

# C6 adapter: Wi-Fi WASD, wheel PI/FF, LCD

## Driving page and network flow (rev19)

Rev19 is installed. Driving buttons, direction keys and a single speed slider
come first; mode settings, Wi-Fi changes and telemetry are collapsed. A connected
network is shown as ready without requesting another password. Explicit enable
clicks now queue behind polling and are cancelled by STOP, blur or stale state;
polling also leaves partially typed speed limits alone.

Network changes work through the existing local page, using the same access and
request-token checks and the stopped-wheel guard on AP and STA. Personal Wi-Fi
passwords can be explicitly reused only for the same saved SSID/auth type; they
are never returned to the browser. Saving an identical connected profile returns
`unchanged: true` without writing NVS or restarting Wi-Fi. This was tested on
Home Network over STA with saved-password reuse: the connection, session and HTTP
restart counter remained unchanged. Bad tokens were rejected. A different real
network/password was not submitted in this verification.

After an unqualified boot the robot selects `floor_trial`, still disarmed, with
unchanged 0.08 m/s and 0.4 rad/s limits and unchanged motor gains. UI race tests,
network parser tests under sanitizers, C6 compilation, and desktop/mobile browser
previews passed. Evidence and screenshots: `reports/newboard-drive-ux-01/`.
No arming or motion commands were sent during deployment verification.


## Mode picker repair (rev18)

Rev18 is installed on the robot. The original mode selector used to reset its
value on every telemetry response and discard changes while a request was in
flight. Committed choices now wait for that request; polling preserves the
focused picker. STOP, reboot, read failure and an armed state cancel pending
changes. A new status must confirm mode/limits before the page can arm.

The regression reproduces on the frozen rev17 page and passes on rev18,
including slow replies, rapid reselection, rejection and STOP races. The served
robot page matches the tested source; five successive Wi-Fi status frames
confirmed `floor_trial`, manual access enabled, disarmed and all outputs zero.
No arming or motion commands were sent for this update. Motor gains and speed
limits are unchanged. Evidence: `reports/newboard-mode-picker-01/`.


## Manual floor trial access (rev17)

Rev17 adds an explicit `floor_trial` option to the original drive page. It uses the
rev14 wheel gains and coordinated start/stop heading controller unchanged. The
firmware limits requested translation to 0.08 m/s and rotation to 0.4 rad/s,
including diagonal/combined commands. This is a manual floor trial, not a claim
of smooth chassis motion, full-range calibration, or wall holding.

The mode is selected while disarmed and lasts only for the current boot. It does
not set or persist `calibrated`, `robust_validated`, or `hold_validated`. Normal
key release sends zero wheel velocity for coordinated deceleration; space/Esc,
page blur, explicit STOP, network loss, faults and the existing 300 ms command
lease stop output. Raw PWM is rejected in this mode even over USB. Switching out
of it removes the trial heading setting and restores other modes' qualification
checks. `manual_control_available` is distinct from `calibrated` in status.

Sources and binary are frozen in `reports/newboard-manual-floor-01/`. Compilation,
native bounds/watchdog/controller-equivalence checks and the actual page's
JavaScript tests passed. Rev17 was flashed after USB reconnected as
`/dev/cu.usbmodem101`; flash contents were verified. USB zero-command timeout and
Wi-Fi arm/zero-command/STOP checks passed. Four fresh status frames confirmed
zero output and stopped wheels. The original no-login page at
`http://192.168.1.x/` is ready in `floor_trial`, disarmed for the user to enable.
No nonzero motion commands were sent during this handoff. See `deployment.json`,
`usb-verification.json` and `web-verification.json` for evidence.


**Current replacement-board floor work, 2026-10-03:**
[Real floor results and English figures](portfolio/newboard-floor/index.html).
**Chassis smoothness has not passed.** The user's latest observation is slight
improvement, but the body still turns at startup and at the end.

The running application has been restored to **`robust-wheels-14-coordinated-stop`**,
SHA256 `a1b348d2b9c0214278ba961628f93e65de876f4b35533c212ec95606cea70c41`.
[Verified stopped restoration](reports/newboard-floor-heading-04/rollback-verified.json).
Source matches the frozen rev14 source. USB development options default OFF;
no full-range wireless or wall-hold validation flags have been written.

The implemented experiment combines moving encoder heading correction with a
normal zero-command deceleration target of 200 ms. Coordination ends after
40 ms of quiet feedback; failure to settle within 550 ms faults. Explicit STOP,
lease expiry and other faults still clear outputs immediately. It retains the
continuous loaded PI and 600-duty/s actuator slew, with no separate startup pulses.
The nominal moving-range models are still the earlier suspended models, not an
identified floor plant. Heading correction does not detect chassis slip.

The first same-binary lateral comparison reduced final encoder yaw from
3.40 to 0.99 degrees left, and 2.90 to 0.60 degrees right. Six subsequent
candidate captures across the four directions had final errors at most
0.66 degrees, but peak yaw during a run reached **2.80 degrees**. The user's
remaining complaint is consistent with this transient error. These are short
0.08 m/s requests, not continuous-motion or chassis-position qualification.
[First comparison](reports/newboard-floor-heading-02/heading-results.json),
[all repeats](reports/newboard-floor-heading-03/heading-results.json).

**Rejected follow-ups are retained:** rev15 tested moving the bounded heading
correction after the command filter. A left trial reached 3.11 degrees peak
versus 0.75 in its paired filtered baseline, exceeding the predefined +1-degree
regression allowance. The remaining right trial was not run. The feature was
restored OFF and the exact rev14 application restored.
[Decision](reports/newboard-floor-heading-04/decision.json).
An earlier version without acceleration feedforward failed offline screening.
Faster duty reduction was also investigated offline: a 1200/s setting failed;
a rate sweep selected 900/s, but independent assumed-model screening encountered
an existing-controller NO_ENCODER case before candidate comparison. This does
not prove a real hardware failure or a robust candidate. It was **not flashed**.
[Retained screening](reports/newboard-duty-reduction-screen-01/decision.json).

The subsequent data collection used the existing rev14 firmware: a 0.08 m/s launch,
then one motor's +/-8/255 input perturbation while the other duties remain fixed.
Each request sequence is 1.4 seconds with 14 cm encoder excursion and 0.25 rad
heading bounds, raw duty <=130, raw speed <=3500 counts/s, fresh telemetry and
verified stopping. This is independent input-output data, not a new calibration
or evidence of smooth chassis motion.
[Prospective plan](reports/newboard-floor-excitation-02/manifest.json).

Six training captures completed; six independent validation captures were
attempted, with five complete and one aborted before raw input because M2's
baseline duty exceeded the experiment's admission bound. The user confirmed
the cable was loose, wheels unobstructed and no hand contact. **None of the
five completed validations met the predefined model criteria.** This is not
six completed failures, nor a valid set of gains to deploy. All attempts retain
1847 deduplicated control samples, including the aborted launch.
[Independent validation](reports/newboard-floor-excitation-validation-02/model-validation.json),
[English plots](portfolio/newboard-floor/loaded-model/index.html).

Six repeats per wheel/sign also show variable motion onset: M1 negative
124–265 ms (PWM near detection 78–149/255), M2 positive 135–249 ms
(84–150/255). These values are not measured static breakaway torque: speed
filtering and chassis-driven wheel motion also affect onset detection.
They do not justify a fixed per-wheel delay offset or a diagnosis of bad
encoders. Supply, contact friction and chassis coupling remain unseparated.
[Startup evidence and hashes](reports/newboard-floor-excitation-validation-02/startup-variation.json).

The feedback design follows the distinction between wheel synchronization and
unobservable external motion in [Feng, Koren and Borenstein's original paper](https://websites.umich.edu/~ykoren/uploads/Cross-Coupling_Motion_Controller_for_Mobile_Robots.pdf).
The implementation is an adaptation to this robot's three-wheel kinematics;
the paper does not validate these gains, this hardware, or wall holding.
Actuator-limited PI uses final-output tracking, consistent with
[MathWorks' anti-windup guidance](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html).

Recent physical comparisons, each at a nominal 0.04 m/s for about two seconds:

- **rev10:** the rightward wheel criteria passed, with last-second means close
  to 100%, but M1 startup peaked at 177% and the user still felt jitter.
- **rev11, gain scheduling:** rejected; M1/M3 slowed severely and M3 last-second
  speed standard deviation reached 75% of target. Native/simplified simulation
  passes did not predict this real failure. The source and capture are retained.
- **rev12, low-speed edge-period estimator:** rejected and rolled back. M1/M3
  peaks were about 203%/200%; M3 last-second standard deviation was 20.006%,
  exceeding the prospectively declared 20%. The user again reported jitter.
  A separate pre-arm `WIFI_CONNECTING` attempt is retained with no driven data.
  The estimator's synthetic quantization test passed; it did not establish a
  real improvement. Experimental header/test are retained but not used by the
  restored main firmware.

[rev10 scores](reports/newboard-floor-response-07/scored-results.json),
[rev11 scores](reports/newboard-floor-response-08/scored-results.json),
[rev12 scores](reports/newboard-floor-response-09/scored-results.json).
None of these single-direction comparisons completed four-direction validation.

An earlier fixed-duty diagnostic was felt to be somewhat better, not accepted
as sufficiently smooth. A subsequent bounded independent-input pilot perturbed
M1 by +/-4/255 after a PI launch, holding the other two motor duties constant.
M3 fell from roughly 440 counts/s to near zero at a constant 80/255 duty. This
is not a suitable steady moving-range model identification or evidence that
all remaining variation is mechanical. Actual voltage, current, chassis motion
have not been measured; the user confirmed only the three wheels touch the floor. The retained
[pilot data](reports/newboard-floor-excitation-01/01-left-m1.json) includes the
stop transient. `scripts/c6_floor_excitation.py` defaults to an offline plan;
execution checks fresh telemetry, duty/direction/speed limits, encoder travel,
heading and verified stopping. It changes no firmware gains or validation flags.

The identification approach follows [MathWorks' input/output data and independent
model-validation requirements](https://www.mathworks.com/help/ident/gs/about-system-identification.html).
Short closed-loop tracking captures and a good one-step fit alone are insufficient
to identify reliable loaded dynamics. The rejected low-speed estimator follows
[TI's period-versus-count measurement rationale](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/08_02_00_28/exports/docs/api_guide_am263x/EXAMPLES_DRIVERS_EQEP_POSITION_SPEED.html).

A speed-region comparison on the same rev10 used **0.08 m/s for one second**
(the same nominal 8 cm travel). The first right/left captures passed the existing
short-response diagnostic. Rightward peaks were about 104%/119%/104%, with
last-400-ms standard deviations 7.6%/3.2%/7.1%; leftward peaks were
101%/99%/124%, standard deviations 3.4%/1.6%/3.4%. This is an exploratory
speed-region comparison, not evidence that the controller is now smooth on
all surfaces. Four prospectively reordered repeats also passed the short wheel
criteria, but onset spread still reached 140 ms and single-step encoder heading
change reached 4.7 degrees. This leaves startup synchronization and straightness
unresolved. [Repeat evidence](reports/newboard-floor-speed-validation-01/comparison-metrics.json).
[All speed-region captures](portfolio/newboard-floor/speed-range/floor-response.html).

Historical **rev9** passed 4/4 exploratory and 8/8 independent low-speed wheel
criteria, but the user reported unacceptable chassis jitter. The repeat maximum
mean error was 5.64% and maximum standard deviation 12.55%; lateral encoder
heading changes still reached about 3.7 degrees. These are wheel results, not
independent body straightness or smoothness validation. Previous failed, aborted
and hand-contact records remain linked in the results page; none were relabelled
as passes. The hand-contact annotation is bound to unchanged raw-file hashes.

No calibration, robust/hold validation or controller-save flags were written.
The replacement board boots disarmed, with full-range wireless motion locked;
its no-login page on Home Network remains configured. Higher speeds, continuous
motion, rotation and wall holding are not qualified. Encoder-only feedback
cannot detect all sliding of the chassis relative to the wall.

Telemetry revision 2 captures control and odometry ages against one clock;
packet-build time is included in the floor collector's original freshness
bounds. The first aborted run showed inconsistent sample ages. After the fix,
237 stopped frames had consistent timestamps, maximum combined age 13 ms.

**Direction-fault fix (2026-09-22):** The deployed guard now requires meaningful
actual drive and more than 250 ms of continuous opposite wheel speed in ordinary
driving as well as hold. All three wheels showed correct positive/negative
feedback in short floor pulses. Two deployed floor transitions passed; the
left-to-right trace captured the short coast that would have tripped the former
single-sample guard. [Evidence and limits](reports/direction-guard-findings.md).
Saved gains were preserved; this is not a new loaded tracking or wall-hold validation.

**Software-only development:** [Continuous encoder servo](ENCODER_SERVO.md) now implements shared motion profiling, wheel-position correction and retained bounded load compensation. The final independent MuJoCo floor set passes 24/24 declared motion bounds; broader load and wall sets retain failures. It is not deployed, selectable from the driving page, or qualified for wall holding. English plots and a simulation video are linked in the study.

**Field failure reported:** the user reports poor straight-line travel and downward slipping on a vertical board, with `ROBUST_INPUT_FAULT` shown in the deployed UI. Suspended results below do not establish usable loaded control. The diagnostic update records the fault type, motor and pre-stop state; it was included in the 2026-09-22 Wi-Fi deployment, but fault capture under motor load remains untested. See [incident and diagnostic scope](reports/wall-control-incident.md).

**Encoder-relative hold update:** [Implementation and retained physical failures](ENCODER_HOLD.md). The new mode adds odometry, heading correction, and release-to-hold at **up to 0.08 m/s / 0.6 rad/s**. The faster prototype failed hold settling, so firmware and UI enforce this limit. The wheel-speed mode retains its adjustable maximum. Neither mode provides independent body-position feedback or verified wall holding. A USB-validated controller selection can now be saved for the exact firmware build; reboot always remains disarmed.

This is a separate runtime for the **existing Waveshare ESP32-C6-LCD-1.47 + adapter + bottom board**. It does not change or unlock the integrated-board review firmware. Physical wheel order is 1,2,3: GPIO pairs 0/1,4/5,2/3. Quadrature polarity is +,+,-, based on this robot's observed forward response.

## Connect

**Replacement screen board, 2026-10-03:** the robot now joins **`Home Network`**
(exact capitalization and space) at **http://192.168.1.x/**. Provisioning used
the new USB-only personal-Wi-Fi command, without switching the Mac's network.
The user's requested **no-login access on Home Network is now deployed** and
verified with the actual page plus 30/30 unauthenticated status reads. A boot
disconnect also exposed and prompted a fix for HTTP socket recovery after Wi-Fi
reconnection. [Latest verification](reports/home-network-no-login-verified.json).
All 30 authenticated LAN status reads passed (67.9 ms median, 140.9 ms maximum);
the robot remained stopped. Wi-Fi settings are saved, but this replacement board
is **not calibrated** and no motion validation was unlocked. See
[connection evidence](reports/home-network-usb-provision-20261003.json) and
[USB provisioning instructions](WIFI_NETWORK.md#usb-provisioning-without-switching-the-computers-wi-fi).

**Previous board's verified connection:** computer and robot used the user's **Owner Hotspot**
password hotspot. Open **http://172.20.10.x/** directly; that exact personal
hotspot no longer requires a second browser login, as requested. The update is
flashed: the actual page and 30/30 status reads worked without credentials
(39.6 ms median, 65.2 ms maximum round-trip). Motor outputs remained zero and
saved calibration was retained. This verifies wireless access, not loaded motion
or wall holding. [Evidence](reports/iphone-no-login-deploy.json).

**Wi-Fi update flashed 2026-09-22:** the original driving page now includes local Wi-Fi provisioning for personal passwords and PEAP/TTLS school accounts, with required CA/server verification for enterprise networks. USB verification confirmed stopped outputs, LCD initialization and unchanged legacy gains. Connect to `WBB-C6-XXXX` and open `http://192.168.4.1/`. Enter school credentials only in the local form. An actual robot school login and LAN reachability remain unverified. Robust/hold validation reset for the new build, so the robot currently starts in calibrated legacy mode. Captive-portal login and client-certificate enrollment are not supported. See [network setup and deployment evidence](WIFI_NETWORK.md).

Join the `WBB-C6-…` Wi-Fi network shown on the LCD. The generated 12-character WPA password is also shown on the LCD and persists across resets. Open **http://192.168.4.1**. There is no internet service on this hotspot and no cloud dependency. The hotspot permits one station.

After calibration, select a mode while stopped and click Enable control. Hold W/S for forward/back, A/D for lateral motion, Q/E for rotation. Space/Esc locks control. Pointer buttons also support press-and-hold. In wheel-speed modes, release coasts to a stop. In encoder hold mode, release starts encoder-relative correction, which may take seconds. Leaving the window, hiding the tab, or losing commands for 300ms revokes the lease in every mode. Press the board's BOOT button to stop. These are software stops, not physical motor-power isolation.

The speed-control update defaults to **0.18m/s** (previously 0.08m/s). Set **Speed limit** between 0.04 and 0.40m/s, then use the current-speed slider or **Use max speed**. The initial maximum is 0.30m/s. Lowering the maximum also lowers the selected speed when necessary. Settings are remembered in the current browser, with defaults if storage is unavailable; reloading never arms the robot. Typing in the maximum field cannot trigger WASD/QE, and focusing a speed field clears held motion keys. Combined translation/rotation scales all wheel commands together at the firmware limit of 10000 counts/s, preserving their ratios. The UI falls back to 8000 counts/s if connected to old firmware. PWM remains bounded to ±255. The integral correction limit is ±100 PWM: a high-speed bench run showed motor 2 saturating the previous ±65 limit with 14.6% mean underspeed at +9500 counts/s. The wider integral bound retains the existing output, feedback and timeout protections. Retesting all three wheels at ±4500/±9500 counts/s passed; mean error was at most 4.3% overall and below 1% at ±9500 (`reports/speed-range-final.json`). High-speed zero-output stopping is coasting: the suspended wheels took roughly one second to come to rest. This changes the requested speed range, not the saved PI gains or the motor's available torque. The update is flashed: live status confirms UI revision 2 and the 10000-count/s ceiling, saved gains were retained, and a zero-command lease-expiry test kept all output pads LOW (`reports/speed-update-live.json`). Combined suspended-wheel tests also passed at the new 0.18m/s default, 0.40m/s lateral command, combined translation/rotation reaching the 10000-count/s wheel cap, and low-speed rotation (`reports/speed-combined-final.json`); all wheels were verified stopped afterward. The expanded speed range has not been qualified under load. Browser storage behavior was checked against [MDN localStorage documentation](https://developer.mozilla.org/en-US/docs/Web/API/Window/localStorage).

**Previous board's bench status:** Three-wheel PI + feedforward parameters have been measured, validated and saved to the board. All 12 individual-wheel tests at ±1800 and ±3000 counts/s passed: maximum mean tracking error 9.59%, maximum speed standard deviation 4.0% of target. See `calibration.json`, `reports/validation-v3b.json` and the raw `.jsonl` telemetry. A hardware reset confirmed all six gain sets persisted, startup was disarmed and all output pads remained LOW (`reports/persisted-final.json`). The live heartbeat-loss test passed, with both motor-control pads LOW after stop (`reports/watchdog-final.json`). The six WASD/QE wheel-target vectors also passed over USB at the previous default speed of 0.08m/s (maximum mean error 5.5% for moving wheels), followed by verified zero PWM, LOW output pads and stopped encoder feedback (`reports/wasd-bench.json`). These are suspended-wheel results; floor heading, loaded performance and the wireless browser-to-motor path still need verification.

Physical motor 1 showed intermittent edge activity with nearly cancelling position counts during an earlier sweep. Subsequent A/B diagnostics and the complete sweep/validation did not reproduce it; the underlying cause is unconfirmed. A feedback-quality guard now stops if two consecutive 100ms windows contain many edges but little net travel. Both A and B edge totals are exposed in telemetry. This is separate from the earlier motor-2 runaway with a TF card inserted. That card's CS GPIO4 and MISO GPIO5 conflict with motor 2; the card must remain removed on this adapter. Disconnected-motor and abnormal-feedback sweeps are excluded from tuning.

LCD uses eight fixed rows of large opaque ASCII text, updated only on change at up to 5Hz by a separate task. The user confirmed the text is now readable. It shows hotspot/password/address, control state, target/measured encoder rates and PWM. Backlight uses sigma-delta at 100/255 pulse density, leaving all six LEDC channels for motors. Supply voltage is unavailable, never fabricated. **Remove the microSD/TF card before powering this adapter:** its SD GPIO4/5 are motor control pins, even if no SD library is initialized. The stop path detaches PWM, selects GPIO low, enables pad input readback and latches a fault if a stopped output reads high. That check cannot replace physical motor-power isolation.

## Build and flash

From the repository root:

```sh
.venv/bin/pio run -d c6_drive -e c6_drive
.venv/bin/pio run -d c6_drive -e c6_drive -t upload --upload-port /dev/cu.usbmodem1101
.venv/bin/python scripts/c6_drive_tune.py --mode status
```

Original 8MB flash is backed up locally in `reports/before-drive-flash.bin` (ignored by Git). Connection information is available over USB with a JSON line `{"id":1,"op":"info"}`. This protocol deliberately differs from the old diagnostic console. The firmware starts disarmed.

## Measured tuning workflow

The [portable robust C++ candidate](ROBUST_CONTROL.md) is implemented and exercised directly in MuJoCo: 60 prospective paired trials passed within a declared simulation envelope (52.8% lower mean task error). A separate, wider stress set failed motion qualification and retains a stall/fall. [English replay and media](portfolio/robust.html) show both outcomes. The [wheel-only adapter](ROBUST_HARDWARE.md) was flashed and passed 17 suspended speed fixtures plus reversal/release/timeout checks. The later [encoder-relative hold firmware](ENCODER_HOLD.md) was deployed and saved for that earlier exact build, with nine passing hold fixtures, three wheel regression checks, and a disarmed reboot check. The 2026-09-22 network build intentionally relocked those modes pending new validation. [Wheel hardware results](reports/robust-hardware-summary.md) remain separate from simulation and hold results. The full simulated slip/body-pose mode still requires independent timestamped body pose.

An [interactive simulator](sim/README.md), [independent MuJoCo contact model](sim/physics/README.md), and [English portfolio gallery](portfolio/index.html) now compare candidate controllers and failure responses. Start `.venv/bin/python scripts/c6_drive_sim.py`, then open `http://127.0.0.1:8088/sim/` or `http://127.0.0.1:8088/portfolio/`. This simulation work does not change the flashed firmware or establish loaded wall performance.

For dynamic identification and the proposed unknown-load/slip compensation architecture, see [System identification and unknown-load compensation](SYSTEM_IDENTIFICATION.md). The new `scripts/c6_drive_sysid.py` captures separate bounded open-loop training and validation sequences; it never writes gains. Existing suspended tracking tests do not establish loaded wall-climbing or gravity-hold capability.

Reconnect motors and their supply with wheels clear of the surface before motion tests. Counts/s avoids pretending that the existing 2800 counts/revolution assumption is newly measured. Browser m/s conversion uses the repository's existing 38mm wheel/2800-count geometry; loaded speed and chassis heading remain to be verified.

```sh
.venv/bin/python scripts/c6_drive_tune.py --mode sweep --report c6_drive/reports/sweep.json
.venv/bin/python scripts/c6_drive_tune.py --mode fit --report c6_drive/reports/gains.json
.venv/bin/python scripts/c6_drive_tune.py --mode validate --gains c6_drive/reports/gains.json --report c6_drive/reports/validation.json --save
.venv/bin/python scripts/c6_drive_tune.py --mode watchdog --report c6_drive/reports/watchdog.json
```

Fit requires three stable, moving measurements per wheel/direction, with >80% net travel per encoder edge; it refuses disconnected-motor and poor-feedback data. Candidate feedforward + PI values must be tested, then adjusted if necessary. High-speed sweep extrapolation overestimated low-speed feedforward on this robot, so the final values were refined using measured low-speed duty and response. D stays zero. Measured startup compensation requests at least 140/255 PWM, subject to the output ramp, for at most 250ms and ends early once measured speed exceeds 500 counts/s. Stop commands always clear this compensation. Saving requires all twelve validation runs to have <10% mean error, <20% speed standard deviation and >80% valid net travel per edge. Candidate gains permit USB bench testing only; Wi-Fi driving unlocks after a successful save. Gains are stored in the MCU's `c6-drive` Preferences namespace.

## Verification

```sh
c++ -std=c++17 -fsanitize=address,undefined -I c6_drive/include c6_drive/tests/test_control.cpp -o /tmp/wbb-drive-test
/tmp/wbb-drive-test
node c6_drive/tests/test_ui.cjs
```

Native tests cover lease replay, expiry, counter wrap, malformed/nonfinite values, stall response, late control loops, output/integral limits, key release, pointer cancel, focus loss and the uncalibrated UI lock. They use simulated data. Live USB reports are stored separately under `reports/`.

References checked for this implementation: [Waveshare pinout and display limits](https://docs.waveshare.com/ESP32-C6-LCD-1.47), [Espressif Wi-Fi API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html), [C6 sigma-delta driver](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32c6/api-reference/peripherals/sdm.html), [Adafruit ST7789 source](https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/Adafruit_ST7789.cpp).
