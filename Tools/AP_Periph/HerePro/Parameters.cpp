#include "HerePro.h"

extern const AP_HAL::HAL &hal;

#ifndef AP_PERIPH_LED_BRIGHT_DEFAULT
#define AP_PERIPH_LED_BRIGHT_DEFAULT 100
#endif

#ifndef MAV_SYSTEM_ID
#define MAV_SYSTEM_ID 3
#endif
/*
 *  AP_Periph parameter definitions
 *
 */

#define GSCALAR(v, name, def) { periph.g.v.vtype, name, Parameters::k_param_ ## v, &periph.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { periph.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&periph.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &periph.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&periph.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&periph.v, {group_info : class::var_info} }

#define DEFAULT_LOG_BITMASK   0xffff

const AP_Param::Info HerePro_FW::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // can node number, 0 for dynamic node allocation
    GSCALAR(can_node,         "CAN_NODE", HAL_CAN_DEFAULT_NODE_ID),

    // can node baudrate
    GSCALAR(can_baudrate,     "CAN_BAUDRATE", 1000000),

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    // trigger bootloader flash
    GSCALAR(flash_bootloader,     "FLASH_BOOTLOADER", 0),
#endif
    
    // GPS driver
    // @Group: GPS_
    // @Path: ../../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS_", AP_GPS),

    GSCALAR(rtcmsource,         "GPS_RTCMSOURCE", 0),
    
    // @Group: COMPASS_
    // @Path: ../../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,         "COMPASS_",     Compass),

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager, "SERIAL",  AP_SerialManager),

    GOBJECT(rtc, "RTC_",  AP_RTC),

    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),

    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    GOBJECT(scripting, "SCR_", AP_Scripting),

    GSCALAR(led_brightness, "LED_BRIGHTNESS", AP_PERIPH_LED_BRIGHT_DEFAULT),

    GSCALAR(led_mode, "LED_MODE", 0),

    GSCALAR(serialpass, "SERIALPASS", 0),

    GSCALAR(testmode, "TESTMODE", 0),

    GSCALAR(canterm1, "CAN1_TERMINATOR", 0),
    GSCALAR(canterm2, "CAN2_TERMINATOR", 0),

    GSCALAR(led_idx, "LED_TEST", -1),
    

    AP_VAREND
};


void AP_Periph_FW::load_parameters(void)
{
    AP_Param::setup_sketch_defaults();

    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad parameter table\n");
        AP_HAL::panic("Bad parameter table");
    }
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {
        // erase all parameters
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
    }

    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
}
