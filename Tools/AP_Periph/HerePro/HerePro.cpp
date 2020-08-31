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

extern const AP_HAL::HAL &hal;

AP_Vehicle& vehicle = *AP_Vehicle::get_singleton();

void HerePro_FW::init()
{
    // always run with watchdog enabled. This should have already been
    // setup by the bootloader, but if not then enable now
    // stm32_watchdog_init();

    stm32_watchdog_pat();

    load_parameters();

    stm32_watchdog_pat();

    can_start();

    // initialise serial manager as early as sensible to get
    // diagnostic output during boot process.  We have to initialise
    // the GCS singleton first as it sets the global mavlink system ID
    // which may get used very early on.
    gcs().init();

    // initialise serial ports
    serial_manager.init();
    gcs().setup_console();

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
    palWriteLine(HAL_GPIO_PIN_LED, 1);
    notify.init();
}


void HerePro_FW::update()
{
    static uint32_t last_50hz;
    uint32_t tnow = AP_HAL::millis();

    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        notify.update();
    }
    can_update();
    hal.scheduler->delay(1);
}
