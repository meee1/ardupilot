#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <wolfmqtt/mqtt_client.h>

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
    int use_tls = 1;
    MqttQoS qos = DEFAULT_MQTT_QOS;
    
    static WOLFSSL_METHOD* mMethod = 0;
    static WOLFSSL_CTX* mCtx       = 0;
    static WOLFSSL* mSsl           = 0;
    static const char* mTlsFile    = NULL;
}

namespace AP {
    AP_TelUTM &TelUTM();
};