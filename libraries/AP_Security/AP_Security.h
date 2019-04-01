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
 *
 * Code by Siddharth Bharat Purohit
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <stdint.h>
#include <npnt.h>
#include "KeyManager.h"

#define HEADER_REGION_LEN 16
#define KEY_REGION_LEN 488

#if ((HEADER_REGION_LEN + KEY_REGION_LEN) != 504) && ((KEY_REGION_LEN % 4) != 0) && ((KEY_REGION_LEN % 4) != 0)
#error "Invalid Security Key region Lengths"
#endif

#define AP_PUBLIC_KEY_FILE HAL_BOARD_STORAGE_DIRECTORY "/pubkey.pem" 

#ifndef AP_NPNT_PERMART_FILE
#define AP_NPNT_PERMART_FILE HAL_BOARD_STORAGE_DIRECTORY "/permissionArtifact.xml"
#endif

class AP_Security {

public:
    AP_Security();

    // get singleton instance
    static AP_Security *get_singleton() {
        return _singleton;
    }
    bool load_permission();
private:
    bool _check_npnt_permission();
    static AP_Security *_singleton;
    bool permission_granted;
    npnt_s npnt_handle;
};

namespace AP {
    AP_Security &security();
};
