#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#include <wolfssl/options.h>
#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/version.h>

#include <wolfssl/ssl.h>
#include <wolfssl/wolfcrypt/asn_public.h>
#include <wolfssl/wolfcrypt/coding.h>
#include <wolfssl/wolfcrypt/hmac.h>

#include <wolfmqtt/mqtt_client.h>
//#include <wolfmqtt/mqtt_packet.h>
//#include <wolfmqtt/version.h>

class AP_TelUTM
{
public:
    // constructor
    AP_TelUTM();   

    // get singleton instance
    static AP_TelUTM *get_singleton() {
        return _singleton;
    }
private:
    static AP_TelUTM *_singleton;
    
    MqttClient client;
    MqttNet net;    
};

namespace AP {
    AP_TelUTM &TelUTM();
};