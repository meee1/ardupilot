#pragma once

#include <AP_Common/AP_Common.h>

// Global parameter class.
//
class Parameters {
public:
    static const uint16_t k_format_version = 2;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_gps,
        k_param_compass,
        k_param_can_node,
        k_param_can_baudrate,
        k_param_notify,
        k_param_serial_manager,
        k_param_flash_bootloader,
        k_param_sysid_this_mav,
        k_param_log_bitmask,
        k_param_scripting,
        k_param_led_brightness,
        k_param_rtc,
        k_param_serialpass,
        k_param_testmode,
        k_param_rtcmsource,
        k_param_canterm1,
        k_param_canterm2,
        k_param_led_idx,
        k_param_led_mode,
    };

    AP_Int8 led_mode;

    AP_Int16 serialpass;

    AP_Int16 format_version;
    AP_Int16 can_node;
    AP_Int32 can_baudrate;

    AP_Int32 log_bitmask;

    AP_Int8 flash_bootloader;

    AP_Int8 led_brightness;
    AP_Int16 sysid_this_mav;
    AP_Int8 testmode;
    AP_Int8 rtcmsource;
    AP_Int8 canterm1;
    AP_Int8 canterm2;
    AP_Int8 led_idx;
    
    Parameters() {}
};

extern const AP_Param::Info var_info[];

