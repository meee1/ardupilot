/*
  MAVLink logfile transfer functions
 */

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


#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h> // for LOG_ENTRY

extern const AP_HAL::HAL& hal;

// We avoid doing log messages when timing is critical:
bool AP_Logger::should_handle_log_message()
{
    if (!WritesEnabled()) {
        // this is currently used as a proxy for "in_mavlink_delay"
        return false;
    }
    if (vehicle_is_armed()) {
        return false;
    }
    return true;
}

/**
   handle all types of log download requests from the GCS
 */
void AP_Logger::handle_log_message(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    if (!should_handle_log_message()) {
        return;
    }
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        handle_log_request_list(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        handle_log_request_data(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_ERASE:
        handle_log_request_erase(link, msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        handle_log_request_end(link, msg);
        break;
    }
}

/**
   handle all types of log download requests from the GCS
 */
void AP_Logger::handle_log_request_list(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);

    return;

    if (_log_sending_link != nullptr) {
       // link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
        return;
    }

    mavlink_log_request_list_t packet;
    mavlink_msg_log_request_list_decode(&msg, &packet);

    _log_num_logs = get_num_logs();
    uint16_t last_log = find_last_log();

    if (_log_num_logs == 0) {
        _log_next_list_entry = 0;
        _log_last_list_entry = 0;        
    } else {
        _log_next_list_entry = packet.start;
        _log_last_list_entry = packet.end;

        if (_log_last_list_entry > last_log) {
            _log_last_list_entry = last_log;
        }
        if (_log_next_list_entry < 1) {
            _log_next_list_entry = last_log - _log_num_logs + 1;
        }
    }

    transfer_activity = TransferActivity::LISTING;
    _log_sending_link = &link;

    handle_log_send_listing();
}


/**
   handle request for log data
 */
void AP_Logger::handle_log_request_data(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);

    return;

    if (_log_sending_link != nullptr) {
        // some GCS (e.g. MAVProxy) attempt to stream request_data
        // messages when they're filling gaps in the downloaded logs.
        // This channel check avoids complaining to them, at the cost
        // of silently dropping any repeated attempts to start logging
      //  if (_log_sending_link->get_chan() != link.get_chan()) {
         //   link.send_text(MAV_SEVERITY_INFO, "Log download in progress");
      //  }
        return;
    }

    mavlink_log_request_data_t packet;
    mavlink_msg_log_request_data_decode(&msg, &packet);

    // consider opening or switching logs:
    if (transfer_activity != TransferActivity::SENDING || _log_num_data != packet.id) {

        uint16_t num_logs = get_num_logs();
        uint16_t last_log = find_last_log();
        if (packet.id > last_log || packet.id < (last_log - num_logs + 1)) {
            // request for an invalid log; cancel any current download
            transfer_activity = TransferActivity::IDLE;
            return;
        }

        uint32_t time_utc, size;
        get_log_info(packet.id, size, time_utc);
        _log_num_data = packet.id;
        _log_data_size = size;

        uint32_t end;
        get_log_boundaries(packet.id, _log_data_page, end);
    }

    _log_data_offset = packet.ofs;
    if (_log_data_offset >= _log_data_size) {
        _log_data_remaining = 0;
    } else {
        _log_data_remaining = _log_data_size - _log_data_offset;
    }
    if (_log_data_remaining > packet.count) {
        _log_data_remaining = packet.count;
    }

    transfer_activity = TransferActivity::SENDING;
    _log_sending_link = &link;

    handle_log_send();
}

/**
   handle request to erase log data
 */
void AP_Logger::handle_log_request_erase(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    // mavlink_log_erase_t packet;
    // mavlink_msg_log_erase_decode(&msg, &packet);

    EraseAll();
}

/**
   handle request to stop transfer and resume normal logging
 */
void AP_Logger::handle_log_request_end(GCS_MAVLINK &link, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(_log_send_sem);
    mavlink_log_request_end_t packet;
    mavlink_msg_log_request_end_decode(&msg, &packet);

    transfer_activity = TransferActivity::IDLE;
    _log_sending_link = nullptr;
}

/**
   trigger sending of log messages if there are some pending
 */
void AP_Logger::handle_log_send()
{
    WITH_SEMAPHORE(_log_send_sem);

    if (_log_sending_link == nullptr) {
        return;
    }
    if (hal.util->get_soft_armed()) {
        // might be flying
        return;
    }
    switch (transfer_activity) {
    case TransferActivity::IDLE:
        break;
    case TransferActivity::LISTING:
        handle_log_send_listing();
        break;
    case TransferActivity::SENDING:
        handle_log_sending();
        break;
    }
}

void AP_Logger::handle_log_sending()
{
    WITH_SEMAPHORE(_log_send_sem);




}

/**
   trigger sending of log messages if there are some pending
 */
void AP_Logger::handle_log_send_listing()
{
    WITH_SEMAPHORE(_log_send_sem);

    return;

}

/**
   trigger sending of log data if there are some pending
 */
bool AP_Logger::handle_log_send_data()
{
    WITH_SEMAPHORE(_log_send_sem);

    return false;

}
