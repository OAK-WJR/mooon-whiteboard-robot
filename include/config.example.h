#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// WhiteBoardBot hardware config (template)
// 3x N20 encoder motors + 3x RZ7899-MS drivers + ESP32
// Three omni wheels at 120°
// PCB v2: 2026-04-02 netlist
//
// Usage:
//   cp include/config.example.h include/config.h
// Then fill in the real WiFi credentials in config.h (WIFI_SSID / WIFI_PASS).
// config.h is ignored by .gitignore and is never committed to the repo.
//
// Tuning tip: the PID / speed / lookahead values below are power-on defaults; at runtime you can
// change them live from the serial console (see the command table at the top of src/main.cpp), then write good values back here.
// ============================================================

// --- Omni wheel parameters (38mm omni wheel) ---
#define WHEEL_DIAMETER      0.038f   // wheel diameter (m), 38mm
#define WHEEL_RADIUS        0.019f   // wheel radius (m), 19mm
#define WHEEL_WIDTH         0.02342f // wheel width (m), 23.42mm
#define ROBOT_RADIUS        0.033738f// wheel center to chassis center distance (m), 33.738mm
#define ENCODER_CPR         700      // N20 encoder 7PPR * 1:100 gearbox = 700 CPR

// --- RZ7899-MS driver (one H-bridge each, 2-pin control: FI + BI) ---
// Forward: FI=PWM, BI=LOW    Reverse: FI=LOW, BI=PWM
// Coast stop: FI=LOW, BI=LOW; RZ7889-MS active brake is both pins HIGH

#ifdef CONFIG_IDF_TARGET_ESP32C6
// ===================== ESP32-C6 (adapter board + Waveshare C6-LCD-1.47) =====================
// Index = physical motor number (motor[0]=Motor1, [1]=Motor2, [2]=Motor3), matching the kinematics.
// Note: the measured pairing of adapter-board headers to base-board sockets is crossed: M0 header -> base H2 (motor 1),
//       M1 header -> base H1 (motor 3), M2 header -> base H3 (motor 2). So pins here are bound in physical motor order,
//       [1] uses the M2 header pins and [2] uses the M1 header pins; the kinematics matrix needs no change.
// C6 pins not to use: GPIO12/13 (USB-Serial/JTAG flashing), GPIO9 (BOOT), 6/7/8/14/15/21/22 (used by the display board)

// Motor 0 = physical Motor1 (via adapter M0 header -> base H2 -> controller1)
#define MOTOR0_FI_PIN       0        // GPIO0
#define MOTOR0_BI_PIN       1        // GPIO1

// Motor 1 = physical Motor2 (via adapter M2 header -> base H3 -> controller2)
#define MOTOR1_FI_PIN       4        // GPIO4
#define MOTOR1_BI_PIN       5        // GPIO5

// Motor 2 = physical Motor3 (via adapter M1 header -> base H1 -> controller3)
#define MOTOR2_FI_PIN       2        // GPIO2
#define MOTOR2_BI_PIN       3        // GPIO3

// --- Encoder pins (A/B phase), also in physical motor order ---
#ifdef WBB_TOP_BOARD_V2
#define ENC0_A_PIN          7        // New integrated board: GPIO16 remains UART TX.
#else
#define ENC0_A_PIN          16       // GPIO16 (UART0 TXD, 100Ω series resistor on adapter board)
#endif
#define ENC0_B_PIN          17       // GPIO17 (UART0 RXD)
#define ENC1_A_PIN          20       // GPIO20  <- M2 header
#define ENC1_B_PIN          23       // GPIO23  <- M2 header
#define ENC2_A_PIN          18       // GPIO18  <- M1 header
#define ENC2_B_PIN          19       // GPIO19  <- M1 header

#else
// ===================== ESP32 (old all-in-one board, kept for fallback) =====================
// Motor 0 (controller1, Motor1)
#define MOTOR0_FI_PIN       32       // GPIO32 - controller1 FI (forward input)
#define MOTOR0_BI_PIN       33       // GPIO33 - controller1 BI (reverse input)

// Motor 1 (controller2, Motor2)
#define MOTOR1_FI_PIN       25       // GPIO25 - controller2 FI (forward input)
#define MOTOR1_BI_PIN       26       // GPIO26 - controller2 BI (reverse input)

// Motor 2 (controller3, Motor3)
#define MOTOR2_FI_PIN       27       // GPIO27 - controller3 FI (forward input)
#define MOTOR2_BI_PIN       14       // GPIO14 - controller3 BI (reverse input)

// --- Encoder pins (A/B phase) ---
// motor0 encoder (Motor1 connector) - confirmed by measurement
#define ENC0_A_PIN          34       // GPIO34 - Motor1 Enc A (input only, no internal pull-up)
#define ENC0_B_PIN          35       // GPIO35 - Motor1 Enc B (input only, no internal pull-up)
// motor1 encoder (Motor2 connector) - v2 board reroutes to GPIO22/23 (regular pins, with internal pull-up)
#define ENC1_A_PIN          22       // GPIO22 - Motor2 Enc A (internal pull-up, no external resistor)
#define ENC1_B_PIN          23       // GPIO23 - Motor2 Enc B (internal pull-up, no external resistor)
// motor2 encoder (Motor3 connector) - confirmed by measurement
#define ENC2_A_PIN          19       // GPIO19 - Motor3 Enc A
#define ENC2_B_PIN          18       // GPIO18 - Motor3 Enc B
#endif

// --- Encoder direction correction ---
// Convention: +1 keeps the direction, -1 inverts it
// Fixes "encoder phase direction" problems; does not change the motor drive direction.
// If only the motor wiring is reversed, you usually see: sign(PWM) and sign(measured speed) agree, but the physical motion is still opposite to what you expect.
#define ENCODER_DIR_M0      (+1)
#define ENCODER_DIR_M1      (+1)
#define ENCODER_DIR_M2      (-1)     // per current debugging: invert the M2 encoder count

// --- PWM parameters ---
#define PWM_FREQUENCY       20000    // PWM frequency (Hz)
#define PWM_RESOLUTION      8        // PWM resolution (bits), 0-255
#define PWM_MAX             255

// --- Motor characteristics (N20 + 1:100 gearbox + 38mm omni wheel + RZ7899 driver) ---
// RZ7899 direct H-bridge, RDS(on)=80mΩ, slightly more efficient than TB6612
// Needs measured calibration: values below are initial estimates
#define MOTOR_MAX_SPEED     0.40f    // max wheel linear speed (m/s), to be calibrated by measurement
#define MOTOR_MIN_SPEED     0.005f   // deadband threshold (m/s); below this, output 0
// Static friction breakaway (measured breakaway PWM, 2026-06-17, open-loop step method): M0=100 M1=140 M2=140.
// Added only while the wheel is stuck (|actual speed|<MOTOR_BREAKAWAY_VEL) and removed once it moves,
// so a constant bias does not overshoot or add rotation while running free.
#define MOTOR_STICTION_M0   100.0f
#define MOTOR_STICTION_M1   140.0f
#define MOTOR_STICTION_M2   140.0f
#define MOTOR_BREAKAWAY_VEL 0.02f

// --- Stall protection ---
// Rule: PWM is already large (pushing hard) but actual speed is about 0 (not moving) for several ticks -> stall.
// When hitting a wall, the pen jams, or a wheel presses on a wire, this stops PID from holding PWM at max for long (the worst case for heat and overcurrent).
#define STALL_PWM_THRESH    200      // only counts as "pushing hard" when |PWM| is above this
#define STALL_VEL_THRESH    0.01f    // |actual wheel speed| below this counts as "not moving" (m/s)
#define STALL_TRIP_TICKS    25       // consecutive ticks to trip (50Hz -> 500ms)
#define STALL_COOLDOWN_MS   1000     // how long to stop after a trip before auto retry
#define STALL_MAX_TRIPS     3        // after this many trips in a row, lock that wheel and wait for the host PC

// Wheel speed sanity gate: fastest measured wheel speed is ~0.38 m/s; anything above this is encoder noise/crosstalk and is dropped.
// Mainly blocks the ghost constant speed (~0.58 m/s) caused by M1 motor PWM crosstalk into ENC1 (GPIO36/39).
#define MAX_WHEEL_VEL       0.45f

// --- FF + PID parameters (per motor) ---
// Note: Kf may need recalibration after changing the driver chip
// Global default = autotune result measured on M1 (kf_true=636)
#define PID_KF              636.0f   // feedforward gain = motor DC gain PWM/(m/s)
#define PID_KP              63.4f    // proportional gain
#define PID_KI              3.9f     // integral gain (removes steady-state error)
#define PID_KD              0.0f     // derivative gain (at 50Hz, Kd>0 easily causes a limit cycle; autotune uses 0)
#define PID_INTEGRAL_LIMIT  250.0f   // I-term output limit

// Separate PID per motor (defaults to the global values first; can be tuned one by one later)
// Separate PID per motor - autotuned (tools/autotune.cpp); each of the three wheels differs.
// Real-robot measured kf_true (open-loop PWM=200, 2026-06-16): M0=504 M1=636 M2=515.
// Kf is set directly to each motor's measured DC gain; Kd=0 avoids the 50Hz limit cycle; Ki removes steady-state error.
// M0: tuned by hand (found with PID Tuner on the robot)
#define PID_KP_M0           500.0f
#define PID_KI_M0           10.0f
#define PID_KD_M0           50.0f
#define PID_KF_M0           470.0f
#define PID_ILIM_M0         250.0f

// M1: gentle closed-loop gains. Note the M1 encoder (ENC1 GPIO36/39) has PWM crosstalk, so closed loop jitters;
// but open loop cannot control speed (runs too fast). The root of this dilemma is the encoder; only a hardware fix (pull-ups + separate wiring) makes it both smooth and accurate.
// For now use closed loop here (speed is controlled); the jitter is the encoder's fault.
#define PID_KP_M1           63.4f
#define PID_KI_M1           3.9f
#define PID_KD_M1           0.0f
#define PID_KF_M1           636.0f
#define PID_ILIM_M1         250.0f

#define PID_KP_M2           108.0f
#define PID_KI_M2           7.7f
#define PID_KD_M2           0.0f
#define PID_KF_M2           515.0f
#define PID_ILIM_M2         250.0f

// --- Control rate ---
#define CONTROL_RATE_HZ     50       // control/odometry update rate
#define CONTROL_PERIOD_MS   (1000 / CONTROL_RATE_HZ)

// --- Operating mode ---
// 0 = USB serial mode (default): no WiFi/UDP; commands over serial, pose over the 'j' stream, tuning with pid_tuner.
//     Does not block at startup, no WiFiUdp errors, and removes ENC1 (GPIO36/39) RF noise.
// 1 = WiFi mode: connects to WIFI_SSID/PASS and talks to the host PC over UDP.
#define USE_WIFI            0

// --- WiFi (fill in real values when USE_WIFI=1) ---
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASS           "YOUR_WIFI_PASSWORD"

// --- Autonomous mode WiFi UDP (talks to host PC whiteboard_app.py) ---
#define UDP_LISTEN_PORT     8888    // ESP32 listen port (receives commands)
#define UDP_REPORT_PORT     8889    // ESP32 report port (status)
#define STATUS_RATE_HZ      10      // status report rate (Hz)

// --- Trajectory tracking ---
#define LOOK_AHEAD_DIST     0.06f   // lookahead distance (m)
#define GOAL_TOLERANCE       0.02f   // arrival distance (m); flipped on the bench with no load it cannot stop within 8mm, 2cm is the sweet spot
#define MAX_LINEAR_SPEED     0.15f   // max linear speed (m/s)
#define KP_LINEAR            2.0f    // linear speed proportional gain
#define MAX_WAYPOINTS        200     // max number of waypoints
// End slowdown: within END_DECEL_DIST of the goal, slow down linearly with distance to MIN_APPROACH_SPEED,
// so it does not overshoot and circle, and the robot creeps into the tolerance and stops.
#define END_DECEL_DIST      0.08f   // end slowdown start distance (m)
#define MIN_APPROACH_SPEED  0.04f   // minimum final speed (m/s)

// Heading hold: actively uses angular speed to lock theta back to 0, cancelling rotation drift so point-to-point moves are straight.
// Off by default (KP_ANGULAR=0 -> omega=0): flipped on the bench with no load, odom theta is not reliable and turning it on makes it spin.
// Once on the ground and the M1 encoder is clean (theta reliable), set to ~3.0 to enable it and straighten the path.
#define KP_ANGULAR          0.0f    // heading hold proportional gain (0=off; set to 3.0 once on the ground and reliable)
#define MAX_ANGULAR_SPEED   1.5f    // max angular speed (rad/s)

// R3 environments force these public defaults before private config.h is read.
// Wireless credentials are provisioned to NVS by the integrated runtime.
// The profile is intentionally unqualified for both three-6V and three-12V sets.
#ifdef WBB_TOP_BOARD_R3
#include "board_profile_r3.h"
#endif

#endif // CONFIG_H
