/**
 * WhiteBoardBot - ESP32 固件入口
 *
 * 三轮全向轮机器人底层控制:
 * - micro-ROS 通信 (WiFi UDP 或 serial transport)
 * - 发布 /odom (nav_msgs/Odometry)
 * - 订阅 /cmd_vel (geometry_msgs/Twist)
 * - 编码器里程计 + 逆运动学 + FF+PID 闭环电机控制
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#ifdef MICRO_ROS_TRANSPORT_WIFI
#include <WiFi.h>
#endif

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#include "config.h"
#include "kinematics.h"
#include "odometry.h"
#include "motor_driver.h"
#include "pid_controller.h"

// ---- 全局对象 ----
static Kinematics      kinematics(WHEEL_RADIUS, ROBOT_RADIUS);
static Odometry         odometry(kinematics, ENCODER_CPR);
static TriMotorDriver   motors;
static PIDController    pid[3];   // 每个电机一个 FF+PID

// ---- micro-ROS 对象 ----
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;

static rcl_publisher_t  odom_pub;
static nav_msgs__msg__Odometry odom_msg;

static rcl_subscription_t cmd_vel_sub;
static geometry_msgs__msg__Twist cmd_vel_msg;

static rcl_timer_t      control_timer;
static rclc_executor_t  executor;

// ---- 控制状态 ----
static unsigned long last_control_us = 0;

// 目标机体速度 (由 cmd_vel 回调更新)
static BodyVelocity target_body = {0, 0, 0};

// 安全超时: 超过此时间没收到 cmd_vel 则停止
// WiFi 有延迟，设大一些避免误触发
#define CMD_VEL_TIMEOUT_MS 1000
static unsigned long last_cmd_vel_ms = 0;

// ============================================================
// cmd_vel 订阅回调 (只存目标, 不直接驱动电机)
// ============================================================
static void cmd_vel_callback(const void* msg_in) {
    const geometry_msgs__msg__Twist* twist =
        (const geometry_msgs__msg__Twist*)msg_in;

    target_body.vx    = (float)twist->linear.x;
    target_body.vy    = (float)twist->linear.y;
    target_body.omega = (float)twist->angular.z;

    last_cmd_vel_ms = millis();
}

// ============================================================
// 控制定时器回调 (固定频率闭环: 里程计 + FF+PID + odom 发布)
// ============================================================
static void control_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;

    // ---- 计算 dt ----
    unsigned long now_us = micros();
    float dt = (float)(now_us - last_control_us) / 1000000.0f;
    last_control_us = now_us;

    // 防止 dt 异常 (首次/溢出)
    if (dt <= 0.0f || dt > 0.1f) dt = (float)CONTROL_PERIOD_MS / 1000.0f;

    // ---- 1. 更新里程计 (获取实际轮速) ----
    odometry.update(dt);

    // ---- 2. 安全超时检查 ----
    bool timed_out = (millis() - last_cmd_vel_ms > CMD_VEL_TIMEOUT_MS);
    if (timed_out) {
        target_body.vx    = 0;
        target_body.vy    = 0;
        target_body.omega = 0;
    }

    // ---- 3. 逆运动学: 目标机体速度 → 目标轮线速度 ----
    WheelVelocity target_wheels = kinematics.inverse(target_body);

    // ---- 4. 获取实际轮线速度 (直接从编码器, 避免 forward→inverse 噪声放大) ----
    const WheelVelocity& actual_wheels = odometry.getWheelVelocity();

    // ---- 5. FF + PID 闭环 ----
    for (int i = 0; i < 3; i++) {
        float target_v = target_wheels.v[i];
        float actual_v = actual_wheels.v[i];

        // 死区: 目标速度太小就直接停
        if (fabsf(target_v) < MOTOR_MIN_SPEED && fabsf(target_body.vx) < MOTOR_MIN_SPEED
            && fabsf(target_body.vy) < MOTOR_MIN_SPEED && fabsf(target_body.omega) < 0.01f) {
            pid[i].reset();
            motors.motor(i).stop();
            continue;
        }

        float pwm_f = pid[i].compute(target_v, actual_v, dt);
        int16_t pwm = (int16_t)pwm_f;
        pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
        motors.motor(i).setSpeed(pwm);
    }

    // ---- 6. 填充并发布 odom 消息 ----
    const Pose2D& pose = odometry.getPose();
    const BodyVelocity& vel = odometry.getVelocity();

    // Header
    unsigned long now_ms = millis();
    odom_msg.header.stamp.sec     = (int32_t)(now_ms / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000);

    // Position
    odom_msg.pose.pose.position.x = pose.x;
    odom_msg.pose.pose.position.y = pose.y;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation (yaw → quaternion)
    float half_theta = pose.theta * 0.5f;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sinf(half_theta);
    odom_msg.pose.pose.orientation.w = cosf(half_theta);

    // Velocity
    odom_msg.twist.twist.linear.x  = vel.vx;
    odom_msg.twist.twist.linear.y  = vel.vy;
    odom_msg.twist.twist.linear.z  = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = vel.omega;

    rcl_ret_t rc = rcl_publish(&odom_pub, &odom_msg, NULL);
    (void)rc;
}

// ============================================================
// micro-ROS 错误处理宏
// ============================================================
#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { \
    Serial.printf("micro-ROS error: %d at %s:%d\n", (int)rc, __FILE__, __LINE__); \
    while(1) { delay(100); } } }

// ============================================================
// setup
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    // ---- 硬件初始化 ----
    motors.begin();
    odometry.begin();
    last_control_us = micros();
    last_cmd_vel_ms = millis();

    // ---- FF + PID 初始化 ----
    const float pid_kp[3]   = {PID_KP_M0, PID_KP_M1, PID_KP_M2};
    const float pid_ki[3]   = {PID_KI_M0, PID_KI_M1, PID_KI_M2};
    const float pid_kd[3]   = {PID_KD_M0, PID_KD_M1, PID_KD_M2};
    const float pid_kf[3]   = {PID_KF_M0, PID_KF_M1, PID_KF_M2};
    const float pid_ilim[3] = {PID_ILIM_M0, PID_ILIM_M1, PID_ILIM_M2};

    for (int i = 0; i < 3; i++) {
        pid[i].setGains(pid_kp[i], pid_ki[i], pid_kd[i], pid_kf[i]);
        pid[i].setOutputLimits(-PWM_MAX, PWM_MAX);
        pid[i].setIntegralLimit(pid_ilim[i]);
        // PWM 变化率限制: 最多 1500 PWM/s → 在 50Hz 控制下每周期最多变化 30 PWM
        pid[i].setSlewRate(1500.0f);
    }

    Serial.println("WhiteBoardBot: hardware + PID initialized.");

    // ---- micro-ROS transport 初始化 ----
#ifdef MICRO_ROS_TRANSPORT_WIFI
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
#else
    set_microros_serial_transports(Serial);
#endif

    allocator = rcl_get_default_allocator();

    // 初始化 support
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // 创建节点
    RCCHECK(rclc_node_init_default(&node, "whiteboardbot", "", &support));

    // 创建 odom publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"));

    // 创建 cmd_vel subscriber (best_effort QoS，兼容所有 publisher)
    RCCHECK(rclc_subscription_init_best_effort(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // 创建控制定时器
    RCCHECK(rclc_timer_init_default(
        &control_timer, &support,
        RCL_MS_TO_NS(CONTROL_PERIOD_MS),
        control_timer_callback));

    // 初始化 executor (1 timer + 1 subscription)
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA));

    // 初始化 odom 消息的 frame_id
    static char odom_frame[] = ODOM_FRAME_ID;
    static char base_frame[] = BASE_FRAME_ID;
    odom_msg.header.frame_id.data    = odom_frame;
    odom_msg.header.frame_id.size    = strlen(odom_frame);
    odom_msg.header.frame_id.capacity = sizeof(odom_frame);
    odom_msg.child_frame_id.data     = base_frame;
    odom_msg.child_frame_id.size     = strlen(base_frame);
    odom_msg.child_frame_id.capacity  = sizeof(base_frame);

    Serial.println("WhiteBoardBot: micro-ROS initialized. Waiting for agent...");
}

// ============================================================
// loop
// ============================================================
void loop() {
    // micro-ROS executor spin (非阻塞, 超时 100ms)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
