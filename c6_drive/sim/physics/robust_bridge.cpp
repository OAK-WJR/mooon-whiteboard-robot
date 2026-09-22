// Text-only native harness for the very same portable C++ controller.
#include "robust_control.h"
#include <iostream>
#include <iomanip>
#include <string>
int main(int argc,char** argv){
 using namespace wbb_robust;
 Config cfg;
 if(argc>1)cfg.lambda=std::stof(argv[1]);
 if(argc>2)cfg.accelerationFF=std::stof(argv[2]);
 if(argc>3)cfg.useObserver=std::stoi(argv[3])!=0;
 Controller controller(cfg);controller.arm(0);
 Input in;int encoder,body,stop;
 std::cout<<std::setprecision(9);
 while(std::cin>>in.now>>in.dt>>in.commandStamp>>in.command.x>>in.command.y>>in.command.w
       >>in.speed[0]>>in.speed[1]>>in.speed[2]>>in.applied[0]>>in.applied[1]>>in.applied[2]
       >>encoder>>body>>in.bodyStamp>>in.bodyPose.x>>in.bodyPose.y>>in.bodyPose.w>>stop){
  in.encoderValid=encoder;in.bodyValid=body;in.estop=stop;
  const auto& o=controller.update(in);
  for(int x:o.pwm)std::cout<<x<<' ';
  for(float x:o.reference)std::cout<<x<<' ';
  for(float x:o.compensation)std::cout<<x<<' ';
  for(float x:o.integral)std::cout<<x<<' ';
  std::cout<<o.estimatedPose.x<<' '<<o.estimatedPose.y<<' '<<o.estimatedPose.w<<' '
           <<o.goal.x<<' '<<o.goal.y<<' '<<o.goal.w<<' '<<name(o.state)<<' '<<o.fault<<' '<<o.slip<<' '<<o.demandScale<<'\n'<<std::flush;
 }
 return std::cin.eof()?0:2;
}
