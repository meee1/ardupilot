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
  simulator connection for ardupilot version of Gazebo
*/

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  SIM_Mavlink HIL simulator
 */
class SIM_Mavlink : public Aircraft {
public:
    SIM_Mavlink(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new SIM_Mavlink(home_str, frame_str);
    }

private:
    SocketAPM socket_in{true};
    uint16_t bind_port = 14560;
    SocketAPM socket_out{true};
    
    const char *target_address = "127.0.0.1";
    const uint16_t target_port = 5762;

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);


    SocketAPM mav_socket { false };
    struct {
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink;
    
    bool connected = false;
    bool seen_heartbeat = false;
    uint8_t vehicle_system_id;
    uint8_t vehicle_component_id;
    
    uint32_t startup_ms = 0;
    uint32_t last_heartbeat_ms = 0;
    double last_timestamp = 0;
};

} // namespace SITL
