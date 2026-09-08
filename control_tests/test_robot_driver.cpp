// Actual controller, path planner, odometry and driver on mocked GPIO/interrupts.
// This does not model interrupt concurrency, real motors or RF transport.
#include <cassert>
#include <iostream>
#include "../src/kinematics.cpp"
#include "../src/pid_controller.cpp"
#include "../src/path_planner.cpp"
#include "../src/trajectory_follower.cpp"
#include "../src/odometry.cpp"
#include "../src/motor_driver.cpp"
#include "../src/robot_controller.cpp"
static constexpr int pins[]={MOTOR0_FI_PIN,MOTOR0_BI_PIN,MOTOR1_FI_PIN,MOTOR1_BI_PIN,MOTOR2_FI_PIN,MOTOR2_BI_PIN};
static void fresh(){
    failAttachPin=failWritePin=failDetachPin=-1;
    levels.fill(LOW);duties.fill(0);attached.fill(false);fakeNow=0;pinOperations.clear();
}
static void stopped(){for(int pin:pins)assert(duties[pin]==0);}
int main(){
    for(int index=0;index<6;++index){
        fresh();RobotController robot;robot.begin();robot.setWheelTest(index/2,.05f);robot.update(.02f);
        assert(duties[pins[(index/2)*2]]>0);
        failWritePin=pins[index];robot.update(.02f);
        assert(robot.getState()==STATE_IDLE);stopped();
        JsonDocument status;robot.fillStatus(status);
        assert(!status["pwm_ready"]&&status["motor_pwm_fault"][index/2].as<int>()==MotorDriver::WriteFailed);
        for(int i=0;i<3;++i)assert(status["pwm_command"][i]==0);
        failWritePin=-1;robot.setWheelTest(index/2,.05f);robot.update(.02f);
        assert(robot.getState()==STATE_IDLE);stopped();
    }
    std::cout<<"Six controller-to-driver fault paths PASS\n";
    for(const char* op:{"area","goto"}){
        fresh();RobotController robot;robot.begin();robot.setWheelTest(0,.05f);robot.update(.02f);
        assert(duties[pins[0]]>0);
        JsonDocument request;request["cmd"]=op;
        // An area is planning-only; a goto at the current pose has no path.
        request["x"]=0;request["y"]=0;request["w"]=.1;request["h"]=.1;
        robot.handleCommand(request);stopped();
        robot.update(.02f);stopped();
    }
    std::cout<<"Two path replacement stop scenarios PASS\n";
}
