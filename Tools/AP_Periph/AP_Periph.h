#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_MSP/AP_MSP.h>
#include <AP_MSP/msp.h>
#include "../AP_Bootloader/app_comms.h"
#include "hwing_esc.h"
#include "canard.h"

#if defined(HAL_PERIPH_NEOPIXEL_COUNT) || defined(HAL_PERIPH_ENABLE_NCP5623_LED)
#define AP_PERIPH_HAVE_LED
#endif

#include "Parameters.h"
#include "ch.h"

/*
  app descriptor compatible with MissionPlanner
 */
extern const struct app_descriptor app_descriptor;

class AP_Periph_FW {
public:
    virtual void init();
    virtual void update();
    Parameters g;

    // CAN methods
    static CanardInstance canard;
    static uint8_t PreferredNodeID;
    static uint8_t transfer_id;

    static void fix_float16(float &f);
    static uint16_t pool_peak_percent();
    static void can_printf(const char *fmt, ...);
    static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer);

    void processTx();
    void processRx();
    void process1HzTasks(uint64_t timestamp_usec);
    void can_wait_node_id(void);

    void handle_allocation_response(CanardInstance* ins, CanardRxTransfer* transfer);
    void handle_get_node_info(CanardInstance* ins, CanardRxTransfer* transfer);
    void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer);
    void handle_param_getset(CanardInstance* ins, CanardRxTransfer* transfer);
    void handle_param_executeopcode(CanardInstance* ins, CanardRxTransfer* transfer);
    virtual void handle_herepro_notify(CanardInstance* isns, CanardRxTransfer* transfer) {}
#ifdef HAL_PERIPH_ENABLE_BUZZER
    void handle_beep_command(CanardInstance* ins, CanardRxTransfer* transfer);
#endif
#ifdef HAL_GPIO_PIN_SAFE_LED
    void handle_safety_state(CanardInstance* ins, CanardRxTransfer* transfer);
#endif
#ifdef HAL_PERIPH_ENABLE_GPS
    void handle_RTCMStream(CanardInstance* ins, CanardRxTransfer* transfer);
#endif
#ifdef AP_PERIPH_HAVE_LED
    virtual void handle_lightscommand(CanardInstance* ins, CanardRxTransfer* transfer);
#endif

    virtual void can_start();
    virtual void can_update();
    virtual void can_mag_update();
    virtual void can_gps_update();
    virtual void can_baro_update();
    virtual void can_airspeed_update();
    virtual void can_rangefinder_update();
#ifdef HAL_GPIO_PIN_SAFE_BUTTON
    virtual void can_safety_button_update();
#endif
    virtual void load_parameters();

    AP_SerialManager serial_manager;

#ifdef HAL_PERIPH_ENABLE_GPS
    AP_GPS gps;
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    Compass compass;
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    AP_Baro baro;
#endif

#ifdef HAL_PERIPH_ENABLE_MSP
    struct {
        AP_MSP msp;
        MSP::msp_port_t port;
        uint32_t last_gps_ms;
        uint32_t last_baro_ms;
        uint32_t last_mag_ms;
    } msp;
    void msp_init(AP_HAL::UARTDriver *_uart);
    void msp_sensor_update(void);
    void send_msp_packet(uint16_t cmd, void *p, uint16_t size);
    void send_msp_GPS(void);
    void send_msp_compass(void);
    void send_msp_baro(void);
#endif
    
#ifdef HAL_PERIPH_ENABLE_ADSB
    virtual void adsb_init();
    virtual void adsb_update();
    virtual void can_send_ADSB(struct __mavlink_adsb_vehicle_t &msg);
    struct {
        mavlink_message_t msg;
        mavlink_status_t status;
    } adsb;
#endif

#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    AP_Airspeed airspeed;
#endif

#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    RangeFinder rangefinder;
#endif

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
    virtual void pwm_irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);
    virtual void pwm_hardpoint_init();
    virtual void pwm_hardpoint_update();
    struct {
        uint8_t last_state;
        uint32_t last_ts_us;
        uint32_t last_send_ms;
        uint16_t pwm_value;
        uint16_t highest_pwm;
    } pwm_hardpoint;
#endif

#ifdef HAL_PERIPH_ENABLE_HWESC
    HWESC_Telem hwesc_telem;
    virtual void hwesc_telem_update();
#endif
    
#if !defined(HAL_BOARD_AP_PERIPH_HEREPRO)
    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];
#endif

    uint32_t last_mag_update_ms;
    uint32_t last_gps_update_ms;
    uint32_t last_baro_update_ms;
    uint32_t last_airspeed_update_ms;
};

#if !defined(HAL_BOARD_AP_PERIPH_HEREPRO)
extern AP_Periph_FW periph;
#endif

extern "C" {
void can_printf(const char *fmt, ...);
}

