#include "GCS_MAVLink.h"
#include "HerePro.h"

static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU
};
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
    MSG_MEMINFO,
    MSG_GPS_RAW,
    MSG_GPS_RTK,
};

static const ap_message STREAM_POSITION_msgs[] = {
    MSG_LOCATION,
    MSG_LOCAL_POSITION
};

static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

const struct AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

uint8_t GCS_MAVLINK_HerePro::sysid_my_gcs() const
{
    return periph.g.sysid_this_mav;
}
