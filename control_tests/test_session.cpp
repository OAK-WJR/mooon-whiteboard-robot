#include <cassert>
#include <cstring>
#include <iostream>
#include "control_session.h"
int main(){
 using control::Link;control::Session s;
 assert(!s.claim(Link::WiFi,1,0,1,0));
 assert(s.claim(Link::WiFi,1,100,1,1000));
 assert(!s.claim(Link::BLE,1,200,2,1001));
 assert(!s.accept(Link::WiFi,1,100,2,1100));
 assert(s.accept(Link::BLE,1,100,3,1101));
 assert(s.accept(Link::WiFi,2,100,3,1101));
 assert(s.accept(Link::WiFi,1,100,2,1102));
 assert(s.lastSeen==1100); // Foreign, replayed or reordered traffic cannot renew.
 assert(!s.expired(1849) && s.expired(1850));
 assert(s.accept(Link::WiFi,1,100,3,1850));
 assert(s.link==Link::None);
 assert(s.claim(Link::BLE,2,200,1,1900));
 assert(s.accept(Link::WiFi,1,100,4,1901)); // No handover/resume from old link.
 s.clear();assert(s.claim(Link::USB,0,42,1,0xffffff00));
 assert(!s.expired(uint32_t(0xffffff00+749)));
 assert(s.expired(uint32_t(0xffffff00+750)));
 control::Framer<12> f;
 for(unsigned char c:std::string("{\"a\":"))assert(f.feed(c)==f.More);
 assert(f.feed('1')==f.More);assert(f.feed('}')==f.More);
 assert(f.feed('\n')==f.Line);assert(!strcmp(f.data(),"{\"a\":1}"));
 for(int i=0;i<11;++i)assert(f.feed('x')==f.More);
 assert(f.feed('x')==f.Overflow); // immediate fault, independent of newline
 for(unsigned char c:std::string("{}"))assert(f.feed(c)==f.More);
 assert(f.feed('\n')==f.More);assert(f.feed(0)==f.Overflow);
 f.reset();assert(f.feed('{')==f.More);assert(f.feed('}')==f.More);assert(f.feed('\n')==f.Line);
 std::cout<<"control session / framing fault simulation PASS\n";
}
