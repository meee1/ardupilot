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

class HerePro_FW : public AP_Periph_FW {
    GCS_HerePro _gcs;
    AP_Notify notify;
public:
    void init() override;
    void update() override;

    // setup the var_info table
    AP_Param param_loader{var_info};
    static const AP_Param::Info var_info[];
};

extern HerePro_FW periph;
