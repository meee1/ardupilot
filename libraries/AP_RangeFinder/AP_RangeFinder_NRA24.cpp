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
#include "AP_RangeFinder_NRA24.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "stdio.h"
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_NRA24Serial::AP_RangeFinder_NRA24Serial(RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager,
                                                               uint8_t serial_instance) :
   AP_RangeFinder_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

/* 
   detect if a insightica rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_NRA24Serial::detect(AP_SerialManager &serial_manager,uint8_t serial_instance)
{
     //return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

const int StartSequence=0xAA;  //数据包帧头
const int EndSequence=0X55;  
const uint16_t Thresholdmin = 1412;
const uint16_t Thresholdmax = 1612; 
// read - return last value measured by sensor
bool AP_RangeFinder_NRA24Serial::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }
    // read any available lines from the lidar
    uint8_t frameOK = 0;   
    uint16_t Range  = 0;                                                //雷达实测距离
    int16_t  nbytes = uart->available();

    AP_RangeFinder_NRA24Serial::send_radiovale();
    while ( nbytes-- > 0 ) {
        if( uart->read() == StartSequence ){                            //判断数据包帧头0xAA    
            string[0] = StartSequence;
            if( uart->read() == StartSequence ){                         //判断数据包帧头0xAA
                string[1] = StartSequence;
                for( int i = 2;i < 14;i++ ){                            //存储数据到数组      
                    string[i] = uart->read();
                }
                CheckSum = string[4]+string[5]+string[6]+string[7]+string[8]+string[9]+string[10];       
                if( string[11] == (CheckSum&0xFF)&&string[12] == EndSequence && string[13] == EndSequence ){         //按照协议对收到的数据进行校验  
                    frameOK = 1; 
                    Range = (string[6]*256+string[7]);                    //cm  
                }
            }
        }
    }

    if (frameOK == 0) 
    {
       return false;
    }
    
    if(Range != 0)
    {
        reading_cm = Range;
    }
    return true;
}

/* 
   update the state of the sensor 
   ****** this is used by the up code of sensors.cpp
*/
void AP_RangeFinder_NRA24Serial::update(void)
{
    if (get_reading(state.distance_cm)) {    //update the data
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) { //若是200ms收不到数据则报告
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

void AP_RangeFinder_NRA24Serial::send_radiovale(void)
{
    rcin[0] = 0x07;
    rcin[1] = 0x5c;
    rcin[8] = 0;

    if ( (RC_Channels::get_radio_in(0) > Thresholdmin && RC_Channels::get_radio_in(0) < Thresholdmax)
         && (RC_Channels::get_radio_in(1) > Thresholdmin && RC_Channels::get_radio_in(1) < Thresholdmax)
         && (RC_Channels::get_radio_in(2) > Thresholdmin && RC_Channels::get_radio_in(2) < Thresholdmax))
    {
        rcin[8] = 1;                  //悬停模式
    }else{
        rcin[8] = 2;                  //非悬停模式
    }
    rcin[2] = ( RC_Channels::get_radio_in(0) & 0xff00 ) >> 8;
    rcin[3] = RC_Channels::get_radio_in(0) & 0x00ff;
    rcin[4] = ( RC_Channels::get_radio_in(1) & 0xff00 ) >> 8;
    rcin[5] = RC_Channels::get_radio_in(1) & 0x00ff;
    rcin[6] = ( RC_Channels::get_radio_in(2) & 0xff00 ) >> 8;
    rcin[7] = RC_Channels::get_radio_in(2) & 0x00ff;
    rcin[9] = 0;
    rcin[10] = 0x55;
    rcin[11] = 0x55;
    for ( uint8_t i = 2; i < 9; i++ ){
        rcin[9] += rcin[i];
    }

    for ( uint8_t i = 0; i < 12; i++ ){
        uart->write(rcin[i]);
    }

}