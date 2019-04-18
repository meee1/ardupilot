#include <stdlib.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AP_RSSI/AP_RSSI.h>

#include "AP_Logger.h"
#include "AP_Logger_File.h"
#include "AP_Logger_MAVLink.h"
#include "LoggerMessageWriter.h"

extern const AP_HAL::HAL& hal;


/*
  write a structure format to the log - should be in frontend
 */
void AP_Logger_Backend::Fill_Format(const struct LogStructure *s, struct log_Format &pkt)
{
    memset(&pkt, 0, sizeof(pkt));
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_MSG;
    pkt.type = s->msg_type;
    pkt.length = s->msg_len;
    strncpy(pkt.name, s->name, sizeof(pkt.name));
    strncpy(pkt.format, s->format, sizeof(pkt.format));
    strncpy(pkt.labels, s->labels, sizeof(pkt.labels));
}

/*
  Pack a LogStructure packet into a structure suitable to go to the logfile:
 */
void AP_Logger_Backend::Fill_Format_Units(const struct LogStructure *s, struct log_Format_Units &pkt)
{
    memset(&pkt, 0, sizeof(pkt));
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_UNITS_MSG;
    pkt.time_us = AP_HAL::micros64();
    pkt.format_type = s->msg_type;
    strncpy(pkt.units, s->units, sizeof(pkt.units));
    strncpy(pkt.multipliers, s->multipliers, sizeof(pkt.multipliers));
}

/*
  write a structure format to the log
 */
bool AP_Logger_Backend::Write_Format(const struct LogStructure *s)
{
    struct log_Format pkt;
    Fill_Format(s, pkt);
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a unit definition
 */
bool AP_Logger_Backend::Write_Unit(const struct UnitStructure *s)
{
    struct log_Unit pkt = {
        LOG_PACKET_HEADER_INIT(LOG_UNIT_MSG),
        time_us : AP_HAL::micros64(),
        type    : s->ID,
        unit    : { }
    };
    strncpy(pkt.unit, s->unit, sizeof(pkt.unit));

    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a unit-multiplier definition
 */
bool AP_Logger_Backend::Write_Multiplier(const struct MultiplierStructure *s)
{
    struct log_Format_Multiplier pkt = {};

    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write the units for a format to the log
 */
bool AP_Logger_Backend::Write_Format_Units(const struct LogStructure *s)
{
    struct log_Format_Units pkt;
    Fill_Format_Units(s, pkt);
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a parameter to the log
 */
bool AP_Logger_Backend::Write_Parameter(const char *name, float value)
{
    struct log_Parameter pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMETER_MSG),
        time_us : AP_HAL::micros64(),
        name  : {},
        value : value
    };
    strncpy(pkt.name, name, sizeof(pkt.name));
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

/*
  write a parameter to the log
 */
bool AP_Logger_Backend::Write_Parameter(const AP_Param *ap,
                                            const AP_Param::ParamToken &token,
                                            enum ap_var_type type)
{
    char name[16];
    ap->copy_name_token(token, &name[0], sizeof(name), true);
    return Write_Parameter(name, ap->cast_to_float(type));
}

// Write an GPS packet
void AP_Logger::Write_GPS(uint8_t i, uint64_t time_us)
{
    const AP_GPS &gps = AP::gps();
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const struct Location &loc = gps.location(i);
    struct log_GPS pkt = {};
    WriteBlock(&pkt, sizeof(pkt));

    /* write auxiliary accuracy information as well */
    float hacc = 0, vacc = 0, sacc = 0;
    gps.horizontal_accuracy(i, hacc);
    gps.vertical_accuracy(i, vacc);
    gps.speed_accuracy(i, sacc);
    struct log_GPA pkt2 = {};
    WriteBlock(&pkt2, sizeof(pkt2));
}


// Write an RCIN packet
void AP_Logger::Write_RCIN(void)
{
    uint16_t values[14] = {};
    rc().get_radio_in(values, ARRAY_SIZE(values));
    struct log_RCIN pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an SERVO packet
void AP_Logger::Write_RCOUT(void)
{
    struct log_RCOUT pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an RSSI packet
void AP_Logger::Write_RSSI()
{
    AP_RSSI *rssi = AP::rssi();
    if (rssi == nullptr) {
        return;
    }

    struct log_RSSI pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Baro_instance(uint64_t time_us, uint8_t baro_instance, enum LogMessages type)
{
    AP_Baro &baro = AP::baro();
    float climbrate = baro.get_climb_rate();
    float drift_offset = baro.get_baro_drift_offset();
    float ground_temp = baro.get_ground_temperature();
    struct log_BARO pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a BARO packet
void AP_Logger::Write_Baro(uint64_t time_us)
{
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const AP_Baro &baro = AP::baro();
    Write_Baro_instance(time_us, 0, LOG_BARO_MSG);
    if (baro.num_instances() > 1 && baro.healthy(1)) {
        Write_Baro_instance(time_us, 1, LOG_BAR2_MSG);
    }
    if (baro.num_instances() > 2 && baro.healthy(2)) {
        Write_Baro_instance(time_us, 2, LOG_BAR3_MSG);
    }
}

void AP_Logger::Write_IMU_instance(const uint64_t time_us, const uint8_t imu_instance, const enum LogMessages type)
{
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f &gyro = ins.get_gyro(imu_instance);
    const Vector3f &accel = ins.get_accel(imu_instance);
    struct log_IMU pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an raw accel/gyro data packet
void AP_Logger::Write_IMU()
{
    uint64_t time_us = AP_HAL::micros64();

    const AP_InertialSensor &ins = AP::ins();

    Write_IMU_instance(time_us, 0, LOG_IMU_MSG);
    if (ins.get_gyro_count() < 2 && ins.get_accel_count() < 2) {
        return;
    }

    Write_IMU_instance(time_us, 1, LOG_IMU2_MSG);

    if (ins.get_gyro_count() < 3 && ins.get_accel_count() < 3) {
        return;
    }

    Write_IMU_instance(time_us, 2, LOG_IMU3_MSG);
}

// Write an accel/gyro delta time data packet
void AP_Logger::Write_IMUDT_instance(const uint64_t time_us, const uint8_t imu_instance, const enum LogMessages type)
{
    const AP_InertialSensor &ins = AP::ins();
    float delta_t = ins.get_delta_time();
    float delta_vel_t = ins.get_delta_velocity_dt(imu_instance);
    float delta_ang_t = ins.get_delta_angle_dt(imu_instance);
    Vector3f delta_angle, delta_velocity;
    ins.get_delta_angle(imu_instance, delta_angle);
    ins.get_delta_velocity(imu_instance, delta_velocity);

    struct log_IMUDT pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_IMUDT(uint64_t time_us, uint8_t imu_mask)
{
    const AP_InertialSensor &ins = AP::ins();
    if (imu_mask & 1) {
        Write_IMUDT_instance(time_us, 0, LOG_IMUDT_MSG);
    }
    if ((ins.get_gyro_count() < 2 && ins.get_accel_count() < 2) || !ins.use_gyro(1)) {
        return;
    }

    if (imu_mask & 2) {
        Write_IMUDT_instance(time_us, 1, LOG_IMUDT2_MSG);
    }

    if ((ins.get_gyro_count() < 3 && ins.get_accel_count() < 3) || !ins.use_gyro(2)) {
        return;
    }

    if (imu_mask & 4) {
        Write_IMUDT_instance(time_us, 2, LOG_IMUDT3_MSG);
    }
}

void AP_Logger::Write_Vibration()
{
    uint64_t time_us = AP_HAL::micros64();
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f vibration = ins.get_vibration_levels();
    struct log_Vibe pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

bool AP_Logger_Backend::Write_Mission_Cmd(const AP_Mission &mission,
                                              const AP_Mission::Mission_Command &cmd)
{
    mavlink_mission_item_int_t mav_cmd = {};
    AP_Mission::mission_cmd_to_mavlink_int(cmd,mav_cmd);
    struct log_Cmd pkt = {};
    return WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger_Backend::Write_EntireMission()
{
    LoggerMessageWriter_WriteEntireMission writer;
    writer.set_logger_backend(this);
    writer.process();
}

// Write a text message to the log
bool AP_Logger_Backend::Write_Message(const char *message)
{
    struct log_Message pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MESSAGE_MSG),
        time_us : AP_HAL::micros64(),
        msg  : {}
    };
    strncpy(pkt.msg, message, sizeof(pkt.msg));
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Power(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    uint8_t safety_and_armed = uint8_t(hal.util->safety_switch_state());
    if (hal.util->get_soft_armed()) {
        // encode armed state in bit 3
        safety_and_armed |= 1U<<2;
    }
    struct log_POWR pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
#endif
}

// Write an AHRS2 packet
void AP_Logger::Write_AHRS2(AP_AHRS &ahrs)
{
    Vector3f euler;
    struct Location loc;
    Quaternion quat;
    if (!ahrs.get_secondary_attitude(euler) || !ahrs.get_secondary_position(loc) || !ahrs.get_secondary_quaternion(quat)) {
        return;
    }
    struct log_AHRS pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a POS packet
void AP_Logger::Write_POS(AP_AHRS &ahrs)
{
    Location loc;
    if (!ahrs.get_position(loc)) {
        return;
    }
    float home, origin;
    ahrs.get_relative_position_D_home(home);
    struct log_POS pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

#if AP_AHRS_NAVEKF_AVAILABLE
void AP_Logger::Write_EKF(AP_AHRS_NavEKF &ahrs)
{
    // only log EKF2 if enabled
    if (ahrs.get_NavEKF2().activeCores() > 0) {
        Write_EKF2(ahrs);
    }
    // only log EKF3 if enabled
    if (ahrs.get_NavEKF3().activeCores() > 0) {
        Write_EKF3(ahrs);
    }
}


/*
  write an EKF timing message
 */
void AP_Logger::Write_EKF_Timing(const char *name, uint64_t time_us, const struct ekf_timing &timing)
{
    Write(name,
              "TimeUS,Cnt,IMUMin,IMUMax,EKFMin,EKFMax,AngMin,AngMax,VMin,VMax",
              "QIffffffff",
              time_us,
              timing.count,
              (double)timing.dtIMUavg_min,
              (double)timing.dtIMUavg_max,
              (double)timing.dtEKFavg_min,
              (double)timing.dtEKFavg_max,
              (double)timing.delAngDT_min,
              (double)timing.delAngDT_max,
              (double)timing.delVelDT_min,
              (double)timing.delVelDT_max);
}

void AP_Logger::Write_EKF2(AP_AHRS_NavEKF &ahrs)
{
    uint64_t time_us = AP_HAL::micros64();
    // Write first EKF packet
    Vector3f euler;
    Vector2f posNE;
    float posD;
    Vector3f velNED;
    Vector3f gyroBias;
    float posDownDeriv;
    Location originLLH;
    ahrs.get_NavEKF2().getEulerAngles(0,euler);
    ahrs.get_NavEKF2().getVelNED(0,velNED);
    ahrs.get_NavEKF2().getPosNE(0,posNE);
    ahrs.get_NavEKF2().getPosD(0,posD);
    ahrs.get_NavEKF2().getGyroBias(0,gyroBias);
    posDownDeriv = ahrs.get_NavEKF2().getPosDownDerivative(0);
    if (!ahrs.get_NavEKF2().getOriginLLH(0,originLLH)) {
        originLLH.alt = 0;
    }
    struct log_EKF1 pkt = {};
    WriteBlock(&pkt, sizeof(pkt));

    // Write second EKF packet
    float azbias = 0;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    Vector3f gyroScaleFactor;
    uint8_t magIndex = ahrs.get_NavEKF2().getActiveMag(0);
    ahrs.get_NavEKF2().getAccelZBias(0,azbias);
    ahrs.get_NavEKF2().getWind(0,wind);
    ahrs.get_NavEKF2().getMagNED(0,magNED);
    ahrs.get_NavEKF2().getMagXYZ(0,magXYZ);
    ahrs.get_NavEKF2().getGyroScaleErrorPercentage(0,gyroScaleFactor);
    struct log_NKF2 pkt2 = {};
    WriteBlock(&pkt2, sizeof(pkt2));

    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    ahrs.get_NavEKF2().getInnovations(0,velInnov, posInnov, magInnov, tasInnov, yawInnov);
    struct log_NKF3 pkt3 = {};
    WriteBlock(&pkt3, sizeof(pkt3));

    // Write fourth EKF packet
    float velVar = 0;
    float posVar = 0;
    float hgtVar = 0;
    Vector3f magVar;
    float tasVar = 0;
    Vector2f offset;
    uint16_t faultStatus=0;
    uint8_t timeoutStatus=0;
    nav_filter_status solutionStatus {};
    nav_gps_status gpsStatus {};
    ahrs.get_NavEKF2().getVariances(0,velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
    ahrs.get_NavEKF2().getFilterFaults(0,faultStatus);
    ahrs.get_NavEKF2().getFilterTimeouts(0,timeoutStatus);
    ahrs.get_NavEKF2().getFilterStatus(0,solutionStatus);
    ahrs.get_NavEKF2().getFilterGpsStatus(0,gpsStatus);
    float tiltError;
    ahrs.get_NavEKF2().getTiltError(0,tiltError);
    int8_t primaryIndex = ahrs.get_NavEKF2().getPrimaryCoreIndex();
    struct log_NKF4 pkt4 = {};
    WriteBlock(&pkt4, sizeof(pkt4));

    // Write fifth EKF packet - take data from the primary instance
    float normInnov=0; // normalised innovation variance ratio for optical flow observations fused by the main nav filter
    float gndOffset=0; // estimated vertical position of the terrain relative to the nav filter zero datum
    float flowInnovX=0, flowInnovY=0; // optical flow LOS rate vector innovations from the main nav filter
    float auxFlowInnov=0; // optical flow LOS rate innovation from terrain offset estimator
    float HAGL=0; // height above ground level
    float rngInnov=0; // range finder innovations
    float range=0; // measured range
    float gndOffsetErr=0; // filter ground offset state error
    Vector3f predictorErrors; // output predictor angle, velocity and position tracking error
    ahrs.get_NavEKF2().getFlowDebug(-1,normInnov, gndOffset, flowInnovX, flowInnovY, auxFlowInnov, HAGL, rngInnov, range, gndOffsetErr);
    ahrs.get_NavEKF2().getOutputTrackingError(-1,predictorErrors);
    struct log_NKF5 pkt5 = {};
    WriteBlock(&pkt5, sizeof(pkt5));

    // log quaternion
    Quaternion quat;
    ahrs.get_NavEKF2().getQuaternion(0, quat);
    struct log_Quaternion pktq1 = {};
    WriteBlock(&pktq1, sizeof(pktq1));

    // log innovations for the second IMU if enabled
    if (ahrs.get_NavEKF2().activeCores() >= 2) {
        // Write 6th EKF packet
        ahrs.get_NavEKF2().getEulerAngles(1,euler);
        ahrs.get_NavEKF2().getVelNED(1,velNED);
        ahrs.get_NavEKF2().getPosNE(1,posNE);
        ahrs.get_NavEKF2().getPosD(1,posD);
        ahrs.get_NavEKF2().getGyroBias(1,gyroBias);
        posDownDeriv = ahrs.get_NavEKF2().getPosDownDerivative(1);
        if (!ahrs.get_NavEKF2().getOriginLLH(1,originLLH)) {
            originLLH.alt = 0;
        }
        struct log_EKF1 pkt6 = {};
        WriteBlock(&pkt6, sizeof(pkt6));

        // Write 7th EKF packet
        ahrs.get_NavEKF2().getAccelZBias(1,azbias);
        ahrs.get_NavEKF2().getWind(1,wind);
        ahrs.get_NavEKF2().getMagNED(1,magNED);
        ahrs.get_NavEKF2().getMagXYZ(1,magXYZ);
        ahrs.get_NavEKF2().getGyroScaleErrorPercentage(1,gyroScaleFactor);
        magIndex = ahrs.get_NavEKF2().getActiveMag(1);
        struct log_NKF2 pkt7 = {};
        WriteBlock(&pkt7, sizeof(pkt7));

        // Write 8th EKF packet
        ahrs.get_NavEKF2().getInnovations(1,velInnov, posInnov, magInnov, tasInnov, yawInnov);
        struct log_NKF3 pkt8 = {};
        WriteBlock(&pkt8, sizeof(pkt8));

        // Write 9th EKF packet
        ahrs.get_NavEKF2().getVariances(1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
        ahrs.get_NavEKF2().getFilterFaults(1,faultStatus);
        ahrs.get_NavEKF2().getFilterTimeouts(1,timeoutStatus);
        ahrs.get_NavEKF2().getFilterStatus(1,solutionStatus);
        ahrs.get_NavEKF2().getFilterGpsStatus(1,gpsStatus);
        ahrs.get_NavEKF2().getTiltError(1,tiltError);
        struct log_NKF4 pkt9 = {};
        WriteBlock(&pkt9, sizeof(pkt9));

        ahrs.get_NavEKF2().getQuaternion(1, quat);
        struct log_Quaternion pktq2 = {};
        WriteBlock(&pktq2, sizeof(pktq2));
    }

    // write range beacon fusion debug packet if the range value is non-zero
    if (ahrs.get_beacon() != nullptr) {
        uint8_t ID;
        float rng;
        float innovVar;
        float innov;
        float testRatio;
        Vector3f beaconPosNED;
        float bcnPosOffsetHigh;
        float bcnPosOffsetLow;
        if (ahrs.get_NavEKF2().getRangeBeaconDebug(-1, ID, rng, innov, innovVar, testRatio, beaconPosNED, bcnPosOffsetHigh, bcnPosOffsetLow)) {
            if (rng > 0.0f) {
                struct log_RngBcnDebug pkt10 = {};
                WriteBlock(&pkt10, sizeof(pkt10));
            }
        }
    }

    // log EKF timing statistics every 5s
    static uint32_t lastTimingLogTime_ms = 0;
    if (AP_HAL::millis() - lastTimingLogTime_ms > 5000) {
        lastTimingLogTime_ms = AP_HAL::millis();
        struct ekf_timing timing;
        for (uint8_t i=0; i<ahrs.get_NavEKF2().activeCores(); i++) {
            ahrs.get_NavEKF2().getTimingStatistics(i, timing);
            Write_EKF_Timing(i==0?"NKT1":"NKT2", time_us, timing);
        }
    }
}


void AP_Logger::Write_EKF3(AP_AHRS_NavEKF &ahrs)
{
    uint64_t time_us = AP_HAL::micros64();
	// Write first EKF packet
    Vector3f euler;
    Vector2f posNE;
    float posD;
    Vector3f velNED;
    Vector3f gyroBias;
    float posDownDeriv;
    Location originLLH;
    ahrs.get_NavEKF3().getEulerAngles(0,euler);
    ahrs.get_NavEKF3().getVelNED(0,velNED);
    ahrs.get_NavEKF3().getPosNE(0,posNE);
    ahrs.get_NavEKF3().getPosD(0,posD);
    ahrs.get_NavEKF3().getGyroBias(0,gyroBias);
    posDownDeriv = ahrs.get_NavEKF3().getPosDownDerivative(0);
    if (!ahrs.get_NavEKF3().getOriginLLH(0,originLLH)) {
        originLLH.alt = 0;
    }
    struct log_EKF1 pkt = {};
    WriteBlock(&pkt, sizeof(pkt));

    // Write second EKF packet
    Vector3f accelBias;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    uint8_t magIndex = ahrs.get_NavEKF3().getActiveMag(0);
    ahrs.get_NavEKF3().getAccelBias(0,accelBias);
    ahrs.get_NavEKF3().getWind(0,wind);
    ahrs.get_NavEKF3().getMagNED(0,magNED);
    ahrs.get_NavEKF3().getMagXYZ(0,magXYZ);
    struct log_NKF2a pkt2 = {};
    WriteBlock(&pkt2, sizeof(pkt2));

    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    ahrs.get_NavEKF3().getInnovations(0,velInnov, posInnov, magInnov, tasInnov, yawInnov);
    struct log_NKF3 pkt3 = {};
    WriteBlock(&pkt3, sizeof(pkt3));

    // Write fourth EKF packet
    float velVar = 0;
    float posVar = 0;
    float hgtVar = 0;
    Vector3f magVar;
    float tasVar = 0;
    Vector2f offset;
    uint16_t faultStatus=0;
    uint8_t timeoutStatus=0;
    nav_filter_status solutionStatus {};
    nav_gps_status gpsStatus {};
    ahrs.get_NavEKF3().getVariances(0,velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
    ahrs.get_NavEKF3().getFilterFaults(0,faultStatus);
    ahrs.get_NavEKF3().getFilterTimeouts(0,timeoutStatus);
    ahrs.get_NavEKF3().getFilterStatus(0,solutionStatus);
    ahrs.get_NavEKF3().getFilterGpsStatus(0,gpsStatus);
    float tiltError;
    ahrs.get_NavEKF3().getTiltError(0,tiltError);
    uint8_t primaryIndex = ahrs.get_NavEKF3().getPrimaryCoreIndex();
    struct log_NKF4 pkt4 = {};
    WriteBlock(&pkt4, sizeof(pkt4));

    // Write fifth EKF packet - take data from the primary instance
    float normInnov=0; // normalised innovation variance ratio for optical flow observations fused by the main nav filter
    float gndOffset=0; // estimated vertical position of the terrain relative to the nav filter zero datum
    float flowInnovX=0, flowInnovY=0; // optical flow LOS rate vector innovations from the main nav filter
    float auxFlowInnov=0; // optical flow LOS rate innovation from terrain offset estimator
    float HAGL=0; // height above ground level
    float rngInnov=0; // range finder innovations
    float range=0; // measured range
    float gndOffsetErr=0; // filter ground offset state error
    Vector3f predictorErrors; // output predictor angle, velocity and position tracking error
    ahrs.get_NavEKF3().getFlowDebug(-1,normInnov, gndOffset, flowInnovX, flowInnovY, auxFlowInnov, HAGL, rngInnov, range, gndOffsetErr);
    ahrs.get_NavEKF3().getOutputTrackingError(-1,predictorErrors);
    struct log_NKF5 pkt5 = {};
    WriteBlock(&pkt5, sizeof(pkt5));

    // log quaternion
    Quaternion quat;
    ahrs.get_NavEKF3().getQuaternion(0, quat);
    struct log_Quaternion pktq1 = {};
    WriteBlock(&pktq1, sizeof(pktq1));
    
    // log innovations for the second IMU if enabled
    if (ahrs.get_NavEKF3().activeCores() >= 2) {
        // Write 6th EKF packet
        ahrs.get_NavEKF3().getEulerAngles(1,euler);
        ahrs.get_NavEKF3().getVelNED(1,velNED);
        ahrs.get_NavEKF3().getPosNE(1,posNE);
        ahrs.get_NavEKF3().getPosD(1,posD);
        ahrs.get_NavEKF3().getGyroBias(1,gyroBias);
        posDownDeriv = ahrs.get_NavEKF3().getPosDownDerivative(1);
        if (!ahrs.get_NavEKF3().getOriginLLH(1,originLLH)) {
            originLLH.alt = 0;
        }
        struct log_EKF1 pkt6 = {};
        WriteBlock(&pkt6, sizeof(pkt6));

        // Write 7th EKF packet
        ahrs.get_NavEKF3().getAccelBias(1,accelBias);
        ahrs.get_NavEKF3().getWind(1,wind);
        ahrs.get_NavEKF3().getMagNED(1,magNED);
        ahrs.get_NavEKF3().getMagXYZ(1,magXYZ);
        magIndex = ahrs.get_NavEKF3().getActiveMag(1);
        struct log_NKF2a pkt7 = {};
        WriteBlock(&pkt7, sizeof(pkt7));

        // Write 8th EKF packet
        ahrs.get_NavEKF3().getInnovations(1,velInnov, posInnov, magInnov, tasInnov, yawInnov);
        struct log_NKF3 pkt8 = {};
        WriteBlock(&pkt8, sizeof(pkt8));

        // Write 9th EKF packet
        ahrs.get_NavEKF3().getVariances(1,velVar, posVar, hgtVar, magVar, tasVar, offset);
        tempVar = fmaxf(fmaxf(magVar.x,magVar.y),magVar.z);
        ahrs.get_NavEKF3().getFilterFaults(1,faultStatus);
        ahrs.get_NavEKF3().getFilterTimeouts(1,timeoutStatus);
        ahrs.get_NavEKF3().getFilterStatus(1,solutionStatus);
        ahrs.get_NavEKF3().getFilterGpsStatus(1,gpsStatus);
        ahrs.get_NavEKF3().getTiltError(1,tiltError);
        struct log_NKF4 pkt9 = {};
        WriteBlock(&pkt9, sizeof(pkt9));

        // log quaternion
        ahrs.get_NavEKF3().getQuaternion(1, quat);
        struct log_Quaternion pktq2 = {};
        WriteBlock(&pktq2, sizeof(pktq2));        
    }
    // write range beacon fusion debug packet if the range value is non-zero
    uint8_t ID;
    float rng;
    float innovVar;
    float innov;
    float testRatio;
    Vector3f beaconPosNED;
    float bcnPosOffsetHigh;
    float bcnPosOffsetLow;
    Vector3f posNED;
     if (ahrs.get_NavEKF3().getRangeBeaconDebug(-1, ID, rng, innov, innovVar, testRatio, beaconPosNED, bcnPosOffsetHigh, bcnPosOffsetLow, posNED)) {
        if (rng > 0.0f) {
            struct log_RngBcnDebug pkt10 = {};
            WriteBlock(&pkt10, sizeof(pkt10));
        }
    }
    // write debug data for body frame odometry fusion
    Vector3f velBodyInnov,velBodyInnovVar;
    static uint32_t lastUpdateTime_ms = 0;
    uint32_t updateTime_ms = ahrs.get_NavEKF3().getBodyFrameOdomDebug(-1, velBodyInnov, velBodyInnovVar);
    if (updateTime_ms > lastUpdateTime_ms) {
        struct log_ekfBodyOdomDebug pkt11 = {};
        WriteBlock(&pkt11, sizeof(pkt11));
        lastUpdateTime_ms = updateTime_ms;
    }

    // log state variances every 0.49s
    static uint32_t lastEkfStateVarLogTime_ms = 0;
    if (AP_HAL::millis() - lastEkfStateVarLogTime_ms > 490) {
        lastEkfStateVarLogTime_ms = AP_HAL::millis();
        float stateVar[24];
        ahrs.get_NavEKF3().getStateVariances(-1, stateVar);
        struct log_ekfStateVar pktv1 = {};
        WriteBlock(&pktv1, sizeof(pktv1));
        struct log_ekfStateVar pktv2 = {};
        WriteBlock(&pktv2, sizeof(pktv2));
    }


    // log EKF timing statistics every 5s
    static uint32_t lastTimingLogTime_ms = 0;
    if (AP_HAL::millis() - lastTimingLogTime_ms > 5000) {
        lastTimingLogTime_ms = AP_HAL::millis();
        struct ekf_timing timing;
        for (uint8_t i=0; i<ahrs.get_NavEKF3().activeCores(); i++) {
            ahrs.get_NavEKF3().getTimingStatistics(i, timing);
            Write_EKF_Timing(i==0?"XKT1":"XKT2", time_us, timing);
        }
    }
}
#endif

void AP_Logger::Write_Radio(const mavlink_radio_t &packet)
{
    struct log_Radio pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a Camera packet
void AP_Logger::Write_CameraInfo(enum LogMessages msg, const AP_AHRS &ahrs, const Location &current_loc, uint64_t timestamp_us)
{
    int32_t altitude, altitude_rel, altitude_gps;
    if (current_loc.relative_alt) {
        altitude = current_loc.alt+ahrs.get_home().alt;
        altitude_rel = current_loc.alt;
    } else {
        altitude = current_loc.alt;
        altitude_rel = current_loc.alt - ahrs.get_home().alt;
    }
    const AP_GPS &gps = AP::gps();
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        altitude_gps = gps.location().alt;
    } else {
        altitude_gps = 0;
    }

    struct log_Camera pkt = {};
    WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write a Camera packet
void AP_Logger::Write_Camera(const AP_AHRS &ahrs, const Location &current_loc, uint64_t timestamp_us)
{
    Write_CameraInfo(LOG_CAMERA_MSG, ahrs, current_loc, timestamp_us);
}

// Write a Trigger packet
void AP_Logger::Write_Trigger(const AP_AHRS &ahrs, const Location &current_loc)
{
    Write_CameraInfo(LOG_TRIGGER_MSG, ahrs, current_loc, 0);
}

// Write an attitude packet
void AP_Logger::Write_Attitude(AP_AHRS &ahrs, const Vector3f &targets)
{
    struct log_Attitude pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void AP_Logger::Write_AttitudeView(AP_AHRS_View &ahrs, const Vector3f &targets)
{
    struct log_Attitude pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Current_instance(const uint64_t time_us,
                                                 const uint8_t battery_instance,
                                                 const enum LogMessages type,
                                                 const enum LogMessages celltype)
{
    AP_BattMonitor &battery = AP::battery();
    float temp;
    bool has_temp = battery.get_temperature(temp, battery_instance);
    struct log_Current pkt = {};
    WriteBlock(&pkt, sizeof(pkt));

    // individual cell voltages
    if (battery.has_cell_voltages(battery_instance)) {
        const AP_BattMonitor::cells &cells = battery.get_cell_voltages(battery_instance);
        struct log_Current_Cells cell_pkt = {};
        for (uint8_t i = 0; i < ARRAY_SIZE(cells.cells); i++) {
            cell_pkt.cell_voltages[i] = cells.cells[i] + 1;
        }
        WriteBlock(&cell_pkt, sizeof(cell_pkt));

        // check battery structure can hold all cells
        static_assert(ARRAY_SIZE(cells.cells) == (sizeof(cell_pkt.cell_voltages) / sizeof(cell_pkt.cell_voltages[0])),
                      "Battery cell number doesn't match in library and log structure");
    }
}

// Write an Current data packet
void AP_Logger::Write_Current()
{
    // Big painful assert to ensure that logging won't produce suprising results when the
    // number of battery monitors changes, does have the built in expectation that
    // LOG_COMPASS_MSG follows the last LOG_CURRENT_CELLSx_MSG
    static_assert(((LOG_CURRENT_MSG + AP_BATT_MONITOR_MAX_INSTANCES) == LOG_CURRENT_CELLS_MSG) &&
                  ((LOG_CURRENT_CELLS_MSG + AP_BATT_MONITOR_MAX_INSTANCES) == LOG_COMPASS_MSG),
                  "The number of batt monitors has changed without updating the log "
                  "table entries. Please add new enums for LOG_CURRENT_MSG, LOG_CURRENT_CELLS_MSG "
                  "directly following the highest indexed fields. Don't forget to update the log "
                  "description table as well.");

    const uint64_t time_us = AP_HAL::micros64();
    const uint8_t num_instances = AP::battery().num_instances();
    for (uint8_t i = 0; i < num_instances; i++) {
        Write_Current_instance(time_us,
                                   i,
                                   (LogMessages)((uint8_t)LOG_CURRENT_MSG + i),
                                   (LogMessages)((uint8_t)LOG_CURRENT_CELLS_MSG + i));
    }
}

void AP_Logger::Write_Compass_instance(const uint64_t time_us, const uint8_t mag_instance, const enum LogMessages type)
{
    const Compass &compass = AP::compass();

    const Vector3f &mag_field = compass.get_field(mag_instance);
    const Vector3f &mag_offsets = compass.get_offsets(mag_instance);
    const Vector3f &mag_motor_offsets = compass.get_motor_offsets(mag_instance);
    struct log_Compass pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a Compass packet
void AP_Logger::Write_Compass(uint64_t time_us)
{
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    const Compass &compass = AP::compass();
    if (compass.get_count() > 0) {
        Write_Compass_instance(time_us, 0, LOG_COMPASS_MSG);
    }

    if (compass.get_count() > 1) {
        Write_Compass_instance(time_us, 1, LOG_COMPASS2_MSG);
    }

    if (compass.get_count() > 2) {
        Write_Compass_instance(time_us, 2, LOG_COMPASS3_MSG);
    }
}

// Write a mode packet.
bool AP_Logger_Backend::Write_Mode(uint8_t mode, uint8_t reason)
{
    struct log_Mode pkt = {};
    return WriteCriticalBlock(&pkt, sizeof(pkt));
}

// Write ESC status messages
//   id starts from 0
//   rpm is eRPM (rpm * 100)
//   voltage is in centi-volts
//   current is in centi-amps
//   temperature is in centi-degrees Celsius
//   current_tot is in centi-amp hours
void AP_Logger::Write_ESC(uint8_t id, uint64_t time_us, int32_t rpm, uint16_t voltage, uint16_t current, int16_t temperature, uint16_t current_tot)
{
    // sanity check id
    if (id >= 8) {
        return;
    }
    struct log_Esc pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a Yaw PID packet
void AP_Logger::Write_PID(uint8_t msg_type, const PID_Info &info)
{
    struct log_PID pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_Origin(uint8_t origin_type, const Location &loc)
{
    uint64_t time_us = AP_HAL::micros64();
    struct log_ORGN pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

void AP_Logger::Write_RPM(const AP_RPM &rpm_sensor)
{
    struct log_RPM pkt = {};
    WriteBlock(&pkt, sizeof(pkt));
}

// Write a rate packet
void AP_Logger::Write_Rate(const AP_AHRS_View *ahrs,
                                     const AP_Motors &motors,
                                     const AC_AttitudeControl &attitude_control,
                                     const AC_PosControl &pos_control)
{
    const Vector3f &rate_targets = attitude_control.rate_bf_targets();
    const Vector3f &accel_target = pos_control.get_accel_target();
    struct log_Rate pkt_rate = {};
    WriteBlock(&pkt_rate, sizeof(pkt_rate));
}

// Write visual odometry sensor data
void AP_Logger::Write_VisualOdom(float time_delta, const Vector3f &angle_delta, const Vector3f &position_delta, float confidence)
{
    struct log_VisualOdom pkt_visualodom = {};
    WriteBlock(&pkt_visualodom, sizeof(log_VisualOdom));
}

// Write AOA and SSA
void AP_Logger::Write_AOA_SSA(AP_AHRS &ahrs)
{
    struct log_AOA_SSA aoa_ssa = {};

    WriteBlock(&aoa_ssa, sizeof(aoa_ssa));
}

// Write beacon sensor (position) data
void AP_Logger::Write_Beacon(AP_Beacon &beacon)
{
    if (!beacon.enabled()) {
        return;
    }
    // position
    Vector3f pos;
    float accuracy = 0.0f;
    beacon.get_vehicle_position_ned(pos, accuracy);

    struct log_Beacon pkt_beacon = {};
    WriteBlock(&pkt_beacon, sizeof(pkt_beacon));
}

// Write proximity sensor distances
void AP_Logger::Write_Proximity(AP_Proximity &proximity)
{
    // exit immediately if not enabled
    if (proximity.get_status() == AP_Proximity::Proximity_NotConnected) {
        return;
    }

    AP_Proximity::Proximity_Distance_Array dist_array {};
    proximity.get_horizontal_distances(dist_array);

    float dist_up;
    if (!proximity.get_upward_distance(dist_up)) {
        dist_up = 0.0f;
    }

    float close_ang = 0.0f, close_dist = 0.0f;
    proximity.get_closest_object(close_ang, close_dist);

    struct log_Proximity pkt_proximity = {};
    WriteBlock(&pkt_proximity, sizeof(pkt_proximity));
}

void AP_Logger::Write_SRTL(bool active, uint16_t num_points, uint16_t max_points, uint8_t action, const Vector3f& breadcrumb)
{
    struct log_SRTL pkt_srtl = {};
    WriteBlock(&pkt_srtl, sizeof(pkt_srtl));
}
