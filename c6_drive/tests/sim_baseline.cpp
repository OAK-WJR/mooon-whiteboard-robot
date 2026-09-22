#include "control.h"
#include <iostream>
#include <iomanip>

// Test adapter only: calls the real firmware wheel controller on supplied inputs.
int main() {
    drive::Wheel wheels[3];
    for(auto& w:wheels)for(auto& g:w.gains)
        if(!(std::cin>>g.kp>>g.ki>>g.kv>>g.ks))return 1;
    int motor;float target,measured,dt;
    std::cout<<std::setprecision(9);
    while(std::cin>>motor>>target>>measured>>dt){
        if(motor<0||motor>2)return 2;
        auto& w=wheels[motor];w.target=target;w.measured=measured;w.update(dt);
        std::cout<<w.output<<' '<<w.ramp<<' '<<w.integral<<'\n';
    }
}
