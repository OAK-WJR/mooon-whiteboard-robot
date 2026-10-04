#include "network_request.h"
#include <cassert>
#include <cstdio>
#include <string>

int main(){
    JsonDocument d;drive::NetworkCredentials c;bool forget;
    d["ssid"]="test-home";d["password"]="test-only-password";
    assert(!drive::parseNetworkRequest(d.as<JsonVariantConst>(),true,c,forget));
    assert(!forget&&!std::strcmp(c.ssid,"test-home"));
    d["password"]="short";
    assert(!std::strcmp(drive::parseNetworkRequest(d.as<JsonVariantConst>(),true,c,forget),"INVALID_WIFI"));
    d["password"]="test-only-password";d["ssid"]=std::string(33,'x');
    assert(drive::parseNetworkRequest(d.as<JsonVariantConst>(),true,c,forget));
    deserializeJson(d,"{\"ssid\":\"test\\u0000other\",\"password\":\"test-only-password\"}");
    assert(drive::parseNetworkRequest(d.as<JsonVariantConst>(),true,c,forget));
    d["ssid"]="test-school";d["auth"]="peap";
    d["username"]="test-user";d["server_name"]="radius.example.test";
    d["ca_pem"]="-----BEGIN CERTIFICATE-----\nfixture\n-----END CERTIFICATE-----";
    assert(!std::strcmp(drive::parseNetworkRequest(d.as<JsonVariantConst>(),true,c,forget),"USB_PERSONAL_WIFI_ONLY"));
    assert(!drive::parseNetworkRequest(d.as<JsonVariantConst>(),false,c,forget));
    d["auth"]=17;
    assert(!std::strcmp(drive::parseNetworkRequest(d.as<JsonVariantConst>(),false,c,forget),"INVALID_AUTH"));
    d.clear();d["forget"]=true;
    assert(!drive::parseNetworkRequest(d.as<JsonVariantConst>(),true,c,forget)&&forget);
    d["forget"]="true";
    assert(drive::parseNetworkRequest(d.as<JsonVariantConst>(),true,c,forget));
    d.clear();d.to<JsonArray>();
    assert(!std::strcmp(drive::parseNetworkRequest(d.as<JsonVariantConst>(),true,c,forget),"BAD_JSON"));
    drive::NetworkCredentials saved;
    std::strcpy(saved.ssid,"test-home");std::strcpy(saved.password,"test-only-password");
    d.clear();d["ssid"]="test-home";d["auth"]="personal";d["reuse_saved_password"]=true;
    assert(!drive::parseNetworkRequest(d.as<JsonVariantConst>(),false,c,forget,&saved));
    assert(!std::strcmp(c.password,saved.password));assert(drive::sameNetwork(c,saved));
    assert(drive::parseNetworkRequest(d.as<JsonVariantConst>(),false,c,forget));
    d["ssid"]="another-home";
    assert(drive::parseNetworkRequest(d.as<JsonVariantConst>(),false,c,forget,&saved));
    d["ssid"]="test-home";d["password"]="replacement-password";
    assert(drive::parseNetworkRequest(d.as<JsonVariantConst>(),false,c,forget,&saved));
    d["reuse_saved_password"]=false;
    assert(!drive::parseNetworkRequest(d.as<JsonVariantConst>(),false,c,forget,&saved));
    assert(!drive::sameNetwork(c,saved));
    d["reuse_saved_password"]="true";
    assert(drive::parseNetworkRequest(d.as<JsonVariantConst>(),false,c,forget,&saved));
    d["reuse_saved_password"]=true;d["password"]="";d["auth"]="peap";
    assert(drive::parseNetworkRequest(d.as<JsonVariantConst>(),false,c,forget,&saved));
    std::puts("PASS: USB/web request validation, size/NUL checks, enterprise boundary and explicit forget");
}
