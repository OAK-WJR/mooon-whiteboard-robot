# Control lease expires during signature check: missed-stop fix

2026-09-08 (America/Los_Angeles). The current budget version is not finalized; this fix is in the runtime firmware shared by all integrated board versions, and does not mean the new PCB has passed review.

`command()` checks the 750ms control deadline on entry, then parses the JSON and checks the signature. If the deadline arrives during this processing, `Session::accept()` clears control ownership and returns `LEASE_EXPIRED`. The old handler replied right away without calling `halt()`; the next loop could not find the expired session either, because owner was already cleared. So the controller could keep its previous running state.

After the fix, when the handler gets `LEASE_EXPIRED` it first calls `halt("HEARTBEAT_TIMEOUT")`, then sends the original error response. The lease is not renewed, and the interface error codes stay the same; the stop path also performs the selected board type's motor-enable-request LOW operation.

The regression uses the real command handler and `Session`, starts the signature check at 749ms on a simulated clock, and injects 1ms of delay into the HMAC test adapter. It covers Wi-Fi, BLE and two clock start points, one of which crosses the 32-bit millisecond counter wraparound. The test directly checks the controller stop call and the running state, not only the error code or that owner was cleared. On the unfixed code, the new assertion fails (process exit -6):

```text
Assertion failed: (!robot.moving&&robot.stops>previousStops)
```

Offline verification to reproduce this fix:

```sh
python3 control_tests/verify_lease_deadline.py --build
```

This command reuses the existing public-source staging flow, skips private configuration, and puts in substitute files that fail the build if they are included. The verification compiles in a temporary directory, does not connect to a serial port or wireless device, and does not upload firmware.

The results this time are saved in [report.json](lease_deadline_results/report.json):

- The native USB configuration and the R3 UART configuration each pass 21 real command/loop scenarios, including the new deadline boundary regression and 10,000 fixed-seed malformed inputs.
- Lease/framing, real loop timing calculations, and 6 transport queue/restart regressions pass.
- Native C++ tests enable AddressSanitizer and UndefinedBehaviorSanitizer; the C6 `top_board_v2_r3` cross-compile passes.

The native tests use stand-ins for the clock, SDK, controller and wireless interfaces; the transport restart regression also runs the real transport source. This proves the control flow of stop handling; it cannot prove physical PWM levels, H-bridge power cut, wireless latency or motor stopping distance. The existing circuit review gate still forbids motion; this change did not open it, and did not complete budget, power supply, charging, wiring harness or real-board acceptance.
