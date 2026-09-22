// Same Control/EncoderServo implementation as embedded C++, no truth-pose input.
#include "control.h"
#include <iostream>
#include <iomanip>
#include <string>
int main(int argc,char** argv){
    drive::Control control;
    if(argc>1&&std::string(argv[1])=="servo")control.selectServo();else control.selectHold();
    float delta[3]={};control.observe(0,delta,.01f);control.arm(0,42);
    uint32_t now=0,seq=0;float dt=0,command[3],measured[3],applied[3];int fresh,stop;
    std::cout<<std::setprecision(9);
    while(std::cin>>now>>dt>>command[0]>>command[1]>>command[2]
          >>delta[0]>>delta[1]>>delta[2]>>measured[0]>>measured[1]>>measured[2]
          >>applied[0]>>applied[1]>>applied[2]>>fresh>>stop){
        for(int i=0;i<3;i++){control.wheel[i].measured=measured[i];control.applied[i]=applied[i];}
        control.observe(now,delta,dt);
        if(stop)control.stop("TEST_STOP");
        if(fresh&&control.armed&&!control.acceptBody(now,42,++seq,command))control.reject("BAD_COMMAND");
        control.tick(now,dt);
        for(auto& w:control.wheel)std::cout<<w.output<<' ';
        for(auto& w:control.wheel)std::cout<<w.ramp<<' ';
        for(auto& w:control.wheel)std::cout<<w.integral<<' ';
        auto p=control.odometry.pose,g=control.odometry.goal;
        std::cout<<p.x<<' '<<p.y<<' '<<p.yaw<<' '<<g.x<<' '<<g.y<<' '<<g.yaw<<' '
                 <<control.reason<<' '<<control.armed<<' '<<(control.servoMode?control.servo.scale:control.robust.scale)<<'\n'<<std::flush;
    }
    return std::cin.eof()?0:2;
}
