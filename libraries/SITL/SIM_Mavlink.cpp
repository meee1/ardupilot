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
  simulator connector for ardupilot version of Gazebo
*/

#include "SIM_Mavlink.h"
#include "SITL.h"

#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

SIM_Mavlink::SIM_Mavlink(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    last_timestamp(0)
{
    memset(&mavlink, 0, sizeof(mavlink));
    
    ::printf("Waiting for HIL sim data on UDP port %u\n",
           (unsigned)bind_port);
    
    socket_in.bind("0.0.0.0", bind_port);
}

/*
  decode and send servos
*/
void SIM_Mavlink::send_servos(const struct sitl_input &input)
{
    if (AP_HAL::millis() < 10000) {
        // simulated aircraft don't appear until 10s after startup. This avoids a windows
        // threading issue with non-blocking sockets and the initial wait on uartA
        return;
    }  
    
    //generate mavlink hil servo packet
    //HilControlsMessage or HilActuatorControlsMessage
    mavlink_hil_controls_t hilctl;
    hilctl.time_usec =  AP_HAL::micros();
    hilctl.roll_ailerons = (input.servos[0] - 1000) /1000.0;
    hilctl.pitch_elevator = (input.servos[1]- 1000)/1000.0;
    hilctl.yaw_rudder = (input.servos[2]- 1000) /1000.0;
    hilctl.throttle = (input.servos[3]- 1000) /1000.0;
    hilctl.aux1 = (input.servos[4]- 1500) * 0.002;
    hilctl.aux2 = (input.servos[5]- 1500) * 0.002;
    hilctl.aux3 = (input.servos[6]- 1500) * 0.002;
    hilctl.aux4 = (input.servos[7]- 1500) * 0.002;
    
    rcin_chan_count = 8;
    
    mavlink_message_t msg;
    uint16_t len;
    
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
            uint8_t saved_seq = chan0_status->current_tx_seq;
        chan0_status->current_tx_seq = saved_seq++;
        len = mavlink_msg_hil_controls_encode(vehicle_system_id,
                                               vehicle_component_id,
                                               &msg, &hilctl);


        uint8_t msgbuf[len];
        len = mavlink_msg_to_send_buffer(msgbuf, &msg);
        if (len > 0) {
            socket_in.send(msgbuf, len);
        }
}

/*

 */
void SIM_Mavlink::recv_fdm(const struct sitl_input &input)
{
    // check for incoming MAVLink messages
    uint32_t wait_time_ms = 0;
    uint8_t buf[10000];
    ssize_t ret;

    while ((ret=socket_in.recv(buf, sizeof(buf), wait_time_ms)) > 0) {
        if (!connected) {
            const char *ip;
            uint16_t port;
            socket_in.last_recv_address(ip, port);
            socket_in.connect(ip, port);
            connected = true;
            printf("Connected to %s:%u\n", ip, (unsigned)port);
        }
        if (mavlink.connected) {
            mav_socket.send(buf,ret);
        }
        
        for (uint8_t i=0; i<ret; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status,
                                          buf[i],
                                          &msg, &status) == MAVLINK_FRAMING_OK) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_ATTITUDE: {
                        mavlink_attitude_t pkt;
                        mavlink_msg_attitude_decode(&msg, &pkt);
                        dcm.from_euler(pkt.roll, pkt.pitch, pkt.yaw);
                        ::printf("ATT");
                    }
                    case MAVLINK_MSG_ID_HIL_GPS:{
                        //::printf("MAVLINK_MSG_ID_HIL_GPS %u - %u\n", (unsigned)msg.sysid, (unsigned)msg.compid);
                        mavlink_hil_gps_t pkt;
                        mavlink_msg_hil_gps_decode(&msg, &pkt);
                        velocity_ef = Vector3f(pkt.vn, pkt.ve, pkt.vd);

                        location.lat = pkt.lat;
                        location.lng = pkt.lon;
                        location.alt = pkt.alt;
                        
::printf("GPS");
                        break;
                    }
                    case MAVLINK_MSG_ID_HIL_SENSOR:{
                        //::printf("MAVLINK_MSG_ID_HIL_SENSOR %u\n", (unsigned)msg.sysid);
                        mavlink_hil_sensor_t pkt;
                    mavlink_msg_hil_sensor_decode(&msg, &pkt);
                         // get imu stuff
                        accel_body = Vector3f(pkt.xacc,
                                              pkt.yacc,
                                              pkt.zacc);

                        gyro = Vector3f(pkt.xgyro,
                                        pkt.ygyro,
                                        pkt.zgyro);
                 /*              
                        Vector3f mag_ef = Vector3f(pkt.xmag*275,
                                        pkt.ymag*275,
                                        pkt.zmag*275);
                               
mag_bf =  mag_ef;
*/
//::printf("INS");
                               
float terminal_rotation_rate = 4*radians(360.0f);
float roll_rate_max = radians(700);
float pitch_rate_max = radians(700);
float yaw_rate_max = radians(700);
                               
    // rotational acceleration, in rad/s/s, in body frame
    Vector3f rot_accel;
    rot_accel.x = 0 * roll_rate_max;
    rot_accel.y = 0 * pitch_rate_max;
    rot_accel.z = 0 * yaw_rate_max;

    // rotational air resistance
    rot_accel.x -= gyro.x * radians(5000.0) / terminal_rotation_rate;
    rot_accel.y -= gyro.y * radians(5000.0) / terminal_rotation_rate;
    rot_accel.z -= gyro.z * radians(400.0)  / terminal_rotation_rate;

                                        
                        //update_dynamics(rot_accel);
                    // auto-adjust to crrcsim frame rate
    double deltat = pkt.time_usec - last_timestamp;
    //time_now_us += deltat * 1.0e6;

    //::printf("%f\n",deltat);
    
    if (deltat < 0.01 && deltat > 0) {
        //adjust_frame_time(1.0/deltat);
    }
    last_timestamp = pkt.time_usec;

    
                        break;
                    }
                }
                
              
            }
        }
    }
}

/*
  update the hil simulation by one time step
 */
void SIM_Mavlink::update(const struct sitl_input &input)
{
    //printf("update\n");
    send_servos(input);
    //printf("update 1\n");
    recv_fdm(input);
    //printf("update 2\n");

    // update magnetic field
    update_mag_field_bf();
    //::printf("update done\n");
    
    if (startup_ms == 0)
        startup_ms = AP_HAL::millis();
    
    if (AP_HAL::millis() < (startup_ms + 10000)) {
        // simulated aircraft don't appear until 10s after startup. This avoids a windows
        // threading issue with non-blocking sockets and the initial wait on uartA
        return;
    }
    if (!mavlink.connected && mav_socket.connect(target_address, target_port)) {
        ::printf("hil connected to %s:%u\n", target_address, (unsigned)target_port);
        mav_socket.set_blocking(false);
        mavlink.connected = true;
    }
    if (!mavlink.connected) {
        printf("mavlink not connected\n");
        return;
    }
    
    uint8_t buf[255];
    ssize_t ret;
    while ((ret=mav_socket.recv(buf, sizeof(buf), 0)) > 0) {
        for (uint8_t i=0; i<ret; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status,
                buf[i],
                &msg, &status) == MAVLINK_FRAMING_OK) {

                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        if (!seen_heartbeat) {
                            seen_heartbeat = true;
                            vehicle_component_id = msg.compid;
                            vehicle_system_id = msg.sysid;
                            ::printf("HIL using srcSystem %u - %u\n", (unsigned)vehicle_system_id, vehicle_component_id);
                        }
                        break;
                    }
                }

                uint16_t len;

                mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
                uint8_t saved_seq = chan0_status->current_tx_seq;
                chan0_status->current_tx_seq = saved_seq++;

                uint8_t msgbuf[255];
                len = mavlink_msg_to_send_buffer(msgbuf, &msg);
                if (len > 0) {
                    socket_in.send(msgbuf, len);
                }

            }
        }
    }
}

} // namespace SITL
