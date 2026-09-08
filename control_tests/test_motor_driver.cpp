// Actual motor driver; the Arduino API is the fault-injection boundary.
#include <cassert>
#include <iostream>
#include "../src/motor_driver.cpp"
static constexpr int pins[]={MOTOR0_FI_PIN,MOTOR0_BI_PIN,MOTOR1_FI_PIN,MOTOR1_BI_PIN,MOTOR2_FI_PIN,MOTOR2_BI_PIN};
static unsigned cases=0;
static void fresh(){
    failAttachPin=failWritePin=failDetachPin=-1;
    levels.fill(LOW);duties.fill(0);attached.fill(false);pinOperations.clear();
}
static void stopped(){for(int pin:pins)assert(duties[pin]==0);}
int main(){
    // Both direction transitions, including failure to turn the previous leg off.
    for(int index=0;index<6;++index)for(int sign:{-1,1}){
        fresh();TriMotorDriver motors;motors.begin();assert(motors.ready());
        motors.setSpeeds(-sign*50,-sign*60,-sign*70);
        failWritePin=pins[index];motors.setSpeeds(sign*50,sign*60,sign*70);
        assert(!motors.ready()&&(motors.motor(index/2).fault()&MotorDriver::WriteFailed));
        stopped();failWritePin=-1;motors.setSpeeds(50,60,70);
        assert(!motors.ready());stopped();++cases;
    }
    for(int index=0;index<6;++index){
        fresh();TriMotorDriver motors;motors.begin();motors.setSpeeds(50,60,70);
        failWritePin=pins[index];motors.stopAll();
        assert(!motors.ready()&&(motors.motor(index/2).fault()&MotorDriver::WriteFailed));
        stopped();++cases;
    }
    for(int index=0;index<6;++index){
        fresh();failAttachPin=pins[index];TriMotorDriver motors;motors.begin();
        assert(!motors.ready()&&(motors.motor(index/2).fault()&MotorDriver::InitFailed));
        assert(!attached[pins[(index/2)*2]]&&!attached[pins[(index/2)*2+1]]);
        failAttachPin=-1;motors.setSpeeds(50,60,70);assert(!motors.ready());stopped();++cases;
    }
    for(int leg=0;leg<2;++leg){
        fresh();MotorDriver motor;motor.begin(pins[0],pins[1],0,1,20000,8);
        assert(motor.setSpeed(leg==0?80:-80));
        failWritePin=failDetachPin=pins[leg];assert(!motor.stop());
        assert((motor.fault()&(MotorDriver::WriteFailed|MotorDriver::DetachFailed))==6);
        // Deliberately keep the mock's PWM active: GPIO-low attempts do NOT prove
        // power isolation when peripheral detach fails. The fault must stay visible.
        assert(duties[pins[leg]]==80&&levels[pins[leg]]==LOW&&!motor.ready());
        pinOperations.clear();motor.begin(pins[0],pins[1],0,1,20000,8);
        for(const auto& op:pinOperations)assert(op.kind!="attach");
        failWritePin=failDetachPin=-1;assert(!motor.stop());
        assert(motor.fault()==6);stopped();++cases;
    }
    for(int resolution:{0,15,255}){
        fresh();MotorDriver motor;motor.begin(pins[0],pins[1],0,1,20000,resolution);
        assert(!motor.ready()&&motor.fault()==MotorDriver::InitFailed);
        for(const auto& op:pinOperations)assert(op.kind!="attach");++cases;
    }
    std::cout<<cases<<" production motor-driver fault scenarios PASS\n";
}
