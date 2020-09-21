/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  HerePro main firmware

  To flash this firmware on Linux use:

     st-flash write build/f103-periph/bin/AP_Periph.bin 0x8006000

 */
#include <AP_HAL/AP_HAL.h>
#include "hal.h"
#include <stdio.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include "HerePro.h"
#include <AP_Vehicle/AP_Vehicle.h>
#include <canard.h>
#include <uavcan/equipment/ahrs/RawIMU.h>
#include <uavcan/equipment/power/CircuitStatus.h>
#include <uavcan/equipment/ahrs/Solution.h>

extern const AP_HAL::HAL &hal;

AP_Vehicle& vehicle = *AP_Vehicle::get_singleton();

void HerePro_FW::init()
{
    // always run with watchdog enabled. This should have already been
    // setup by the bootloader, but if not then enable now
    stm32_watchdog_init();

    stm32_watchdog_pat();

    load_parameters();

    stm32_watchdog_pat();

    // initialise serial manager as early as sensible to get
    // diagnostic output during boot process.  We have to initialise
    // the GCS singleton first as it sets the global mavlink system ID
    // which may get used very early on.
    gcs().init();

    // initialise serial ports
    serial_manager.init();
    gcs().setup_console();

    // Register delay callback
    hal.scheduler->register_delay_callback(scheduler_delay_callback, 1);

    can_start();

    stm32_watchdog_pat();

    printf("Booting %08x:%08x %u/%u len=%u 0x%08x\n",
           app_descriptor.image_crc1,
           app_descriptor.image_crc2,
           app_descriptor.version_major, app_descriptor.version_minor,
           app_descriptor.image_size,
           app_descriptor.git_hash);

    if (hal.util->was_watchdog_reset()) {
        printf("Reboot after watchdog reset\n");
    }

    if (gps.get_type(0) != AP_GPS::GPS_Type::GPS_TYPE_NONE) {
        gps.init(serial_manager);
    }

    if (compass.enabled()) {
        compass.init();
    }

    notify.init();

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();


    // enable leds
    notify.init();
    // power leds
    palWriteLine(HAL_GPIO_PIN_LED, 1); // 4 leds left/right
    palWriteLine(HAL_GPIO_PIN_LED_2, 1); // the rest

    {
        // get analog for loop
        _adc0 = hal.analogin->channel(4);
        _adc1 = hal.analogin->channel(13);
        _adc2 = hal.analogin->channel(14);
        _adc3 = hal.analogin->channel(17);
    }

    // Initialise logging
    log_init();
}

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void HerePro_FW::scheduler_delay_callback()
{
    static uint32_t last_1hz, last_50hz;

    AP_Logger &logger = AP::logger();

    // don't allow potentially expensive logging calls:
    logger.EnableWrites(false);

    const uint32_t tnow = AP_HAL::millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs().send_message(MSG_HEARTBEAT);
        gcs().send_message(MSG_SYS_STATUS);
    }

    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs().update_receive();
        gcs().update_send();
    }

    logger.EnableWrites(true);
}

void HerePro_FW::update()
{
    static uint32_t last_1hz;
    static uint32_t last_50hz;
    static uint32_t last_100hz;
    uint32_t tnow = AP_HAL::millis();

    if (tnow - last_1hz >= 1000)
    {
        last_1hz = tnow;

        hal.gpio->pinMode(1, HAL_GPIO_INPUT);
        uint8_t gpio1 = hal.gpio->read(1);
        hal.gpio->pinMode(2, HAL_GPIO_INPUT);
        uint8_t gpio2 = hal.gpio->read(2);
        //can_printf("gpio %u %u\r\n",gpio1,gpio2);

        float adc1 = _adc0->voltage_average();
        float adc2 = _adc1->voltage_average();
        float adc3 = _adc2->voltage_average(); // CircuitStatus
        float adc4 = _adc3->voltage_average();

        //can_printf("analog vcc(5v) %f vcc2(1.8v) %f bat1 %f bat2 %f\r\n",adc1,adc2,adc3,adc4);

        can_voltage_update(0,adc3);
        can_voltage_update(1,adc4);
        can_voltage_update(2,adc1);
        can_voltage_update(3,adc2);
        can_voltage_update(4,gpio1);
        can_voltage_update(5,gpio2);
        can_imu_update();
    }

    if (tnow - last_50hz >= 20) {
        last_50hz = tnow;
        notify.update();          
    }
    if (tnow - last_100hz >= 10) {
        last_100hz = tnow;  
        // update INS immediately to get current gyro data populated
        ins.update();

        // run EKF state estimator (expensive)
        // --------------------
        // we tell AHRS to skip INS update as we have already done it in fast_loop()
        ahrs.update(true);

        // Inertial Nav
        // --------------------
        //inertial_nav.update(vibration_check.high_vibes);      
    }

    can_update();
    hal.scheduler->delay(2);
}

void HerePro_FW::can_voltage_update(uint32_t index, float value)
{
    uint8_t buffer[UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_MAX_SIZE];

    uavcan_equipment_power_CircuitStatus power1;

    power1.circuit_id = index;
    power1.voltage = value;         

    uint32_t len = uavcan_equipment_power_CircuitStatus_encode(&power1, buffer);

    canardBroadcast(&canard,
                                        UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_SIGNATURE,
                                        UAVCAN_EQUIPMENT_POWER_CIRCUITSTATUS_ID,
                                        &transfer_id,
                                        CANARD_TRANSFER_PRIORITY_LOW,
                                        buffer,
                                        len);
}

void HerePro_FW::can_imu_update(void)
{
    {
        static uint64_t last_imu_time_us = 0;

        uint64_t time_us = AP_HAL::micros64();

        const Vector3f &gyro = ins.get_gyro(0);
        const Vector3f &accel = ins.get_accel(0);
        float int_int = (float)(time_us - last_imu_time_us) / 1000.0;
        uavcan_equipment_ahrs_RawIMU pkt { };
        pkt.integration_interval= int_int;

        last_imu_time_us = time_us;
        pkt.timestamp.usec = time_us;

            // the canard dsdl compiler doesn't understand float16
        for (uint8_t i=0; i<3; i++) {
            pkt.accelerometer_latest[i] = accel[i] ;
            fix_float16(pkt.accelerometer_latest[i]);
        }

            // the canard dsdl compiler doesn't understand float16
        for (uint8_t i=0; i<3; i++) {
            pkt.rate_gyro_latest[i] = gyro[i] ;
            fix_float16(pkt.rate_gyro_latest[i]);
        }

        uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_RAWIMU_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_ahrs_RawIMU_encode(&pkt, buffer);

        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_AHRS_RAWIMU_SIGNATURE,
                        UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }

    {
        uavcan_equipment_ahrs_Solution _att_state {};
        uavcan_Timestamp ts {};
        ts.usec = AP_HAL::micros64();
        _att_state.timestamp = ts;

        Quaternion qt;
        Matrix3f rot = ahrs.get_rotation_body_to_ned();
        qt.from_rotation_matrix(rot);
        _att_state.orientation_xyzw[0] = qt.q1;
        _att_state.orientation_xyzw[1] = qt.q2;
        _att_state.orientation_xyzw[2] = qt.q3;
        _att_state.orientation_xyzw[3] = qt.q4;
        fix_float16(_att_state.orientation_xyzw[0]);
        fix_float16(_att_state.orientation_xyzw[1]);
        fix_float16(_att_state.orientation_xyzw[2]);
        fix_float16(_att_state.orientation_xyzw[3]);
        // TODO: extract from EKF
        //_att_state.orientation_covariance

        Vector3f av = ahrs.get_gyro();
        _att_state.angular_velocity[0] = av.x;
        _att_state.angular_velocity[1] = av.y;
        _att_state.angular_velocity[2] = av.z;
        fix_float16(_att_state.angular_velocity[0]);
        fix_float16(_att_state.angular_velocity[1]);
        fix_float16(_att_state.angular_velocity[2]);
        
        // TODO: extract from EKF
        //_att_state.angular_velocity_covariance

        Vector3f la = ahrs.get_accel_ef();
        _att_state.linear_acceleration[0] = la.x;
        _att_state.linear_acceleration[1] = la.y;
        _att_state.linear_acceleration[2] = la.z;
        fix_float16(_att_state.linear_acceleration[0]);
        fix_float16(_att_state.linear_acceleration[1]);
        fix_float16(_att_state.linear_acceleration[2]);

        // TODO: extract from EKF
        //_att_state.linear_acceleration_covariance

        uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_SOLUTION_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_ahrs_Solution_encode(&_att_state, buffer);

        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_AHRS_SOLUTION_SIGNATURE,
                        UAVCAN_EQUIPMENT_AHRS_SOLUTION_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
}

const struct LogStructure HerePro_FW::log_structure[] = {
    LOG_COMMON_STRUCTURES,
};

void HerePro_FW::log_init(void)
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}
