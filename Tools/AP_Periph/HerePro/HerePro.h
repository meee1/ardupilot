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
#include "../AP_Periph.h"
#include "GCS_Mavlink.h"
#include <AP_Notify/AP_Notify.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Logger/AP_Logger.h>

class HerePro_FW : public AP_Periph_FW {
    GCS_HerePro _gcs;
    AP_Notify notify;
    AP_InertialSensor ins;
    
    AP_Logger logger;
    static const struct LogStructure log_structure[];

    AP_AHRS_NavEKF ahrs;

    // Inertial Navigation
    AP_InertialNav_NavEKF inertial_nav{ahrs};

    AP_Scheduler scheduler;

    AP_HAL::AnalogSource *_adc0;
    AP_HAL::AnalogSource *_adc1;
    AP_HAL::AnalogSource *_adc2;
    AP_HAL::AnalogSource *_adc3;

    bool start_command_callback(const AP_Mission::Mission_Command& cmd) { return false; }
    void exit_mission_callback() { return; }
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd) { return false; }
    void log_init(void);
    static void scheduler_delay_callback();

    //TODO: Remove dependencies on initialisation
    // of following classes in GCS
    // Mission library
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&HerePro_FW::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&HerePro_FW::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&HerePro_FW::exit_mission_callback, void)};

public:
    void init() override;
    void update() override;
    void can_imu_update();
    void can_voltage_update(uint32_t index, float value);
    
    HerePro_FW(void)
        : logger(g.log_bitmask)
    {
    }
    // setup the var_info table
    AP_Param param_loader{var_info};
    static const AP_Param::Info var_info[];
};

extern HerePro_FW periph;
