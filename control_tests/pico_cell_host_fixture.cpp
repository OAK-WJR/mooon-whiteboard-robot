#define main unused_pico_cell_test_main
#include "test_pico_cells.cpp"
#undef main

int main(int argc,char** argv) {
    const std::string mode=argc>1?argv[1]:"monitoring";
    auto& bus=pico_rail_mock::bus;
    levels.fill(HIGH);modes.fill(-99);bus.present=true;bus.cellPresent=mode!="missing";
    storedConfig=R"({"ssid":"public-native-fixture","password":"test-only-password","key":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"})";
    setup();
    if(mode=="not_started"){}
    else if(mode=="missing")waitCell("fault");
    else if(mode=="waking" || mode=="settling" || mode=="detecting" || mode=="configuring" || mode=="awaiting_scan")waitCell(mode.c_str());
    else {
        waitCell("monitoring");
        if(mode=="fault"){bus.cellDevice.ram[0x901b]=3;waitCell("fault");}
        if(mode=="stopped")pico_rails::diagnostics().stop();
        if(mode=="lower_invalid" || mode=="safety"){
            const auto seq=cellReport()["seq"].as<uint32_t>();
            if(mode=="lower_invalid"){bus.cellDevice.lower=1800;bus.cellDevice.stack=5600;}
            else bus.cellDevice.safety[1]=0x40;
            for(unsigned i=0;i<4000 && cellReport()["seq"].as<uint32_t>()==seq;++i)stepCells();
        }
    }
    Serial.tx.clear();const auto req=request("cell_status");command(Link::USB,0,json(req).c_str());
    std::cout<<Serial.tx;return Serial.tx.empty()?1:0;
}
