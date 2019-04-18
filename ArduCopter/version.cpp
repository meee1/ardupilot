/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Copter.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

#include <AP_Common/AP_FWVersion.h>

const AP_FWVersion AP_FWVersion::fwver{
     FW_MAJOR,
    FW_MINOR,
   FW_PATCH,
   FW_TYPE,
#ifndef GIT_VERSION
    THISFIRMWARE,
#else
  THISFIRMWARE " (" GIT_VERSION ")",
   GIT_VERSION,
#endif
#ifdef CHIBIOS_GIT_VERSION
    nullptr,
     nullptr,
   "ChibiOS",
     CHIBIOS_GIT_VERSION,
#endif
};
