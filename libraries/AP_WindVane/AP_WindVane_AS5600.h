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
#pragma once

#include "AP_WindVane_Backend.h"
#include "AP_WindVane.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#define AP_WINDVANE_AS5600_DEFAULT_ADDR   0x36

class AP_WindVane_AS5600 : public AP_WindVane_Backend
{
public:
    // detector
    static AP_WindVane_Backend *detect(AP_WindVane &frontend,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update_direction() override;

private:

    // constructor
    AP_WindVane_AS5600(AP_WindVane &frontend, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // I2C device
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    bool _init(void);
    void _timer(void);

    uint16_t angle;
    bool new_value;
    uint32_t last_read_ms;
    
    // get the reading
    bool get_reading(uint16_t &reading_ang);

private: 
    enum RegisterMap {
        AS5600_ZMCO   = 0x00,
        AS5600_ZPOS   = 0x01,
        AS5600_MPOS   = 0x03,
        AS5600_MANG   = 0x05,
        AS5600_CONF   = 0x07,
        AS5600_STATUS = 0x0B,
        AS5600_RAWANG = 0x0C,
        AS5600_ANGLE  = 0x0E,
        AS5600_AGC    = 0x1A,
        AS5600_MAGNIT = 0x1B
    };

    enum RegisterZMCO {
        ZMCO_MASK  = 0x02,
        ZMCO_SHIFT = 0 
    };
    
    enum RegisterCONF {
        WD_MASK    = 0x01,
        WD_SHIFT   = 13,
        FTH_MASK   = 0x03,
        FTH_SHIFT  = 10,
        SF_MASK    = 0x02,
        SF_SHIFT   = 8,
        PWMF_MASK  = 0x02,
        PWMF_SHIFT = 6,
        OUTS_MASK  = 0x02,
        OUTS_SHIFT = 4,
        HYST_MASK  = 0x02,
        HYST_SHIFT = 2,
        PM_MASK    = 0x02,
        PM_SHIFT   = 0,
    };

    enum PowerMode {
        PM_ALWAYS = 0x00,
        PM_5MS    = 0x01,
        PM_20MS   = 0x02,
        PM_100MS  = 0x03
    };

    enum ReigisterSTATUS {
        MD_MASK  = 0x01,
        MD_SHIFT = 5,
        ML_MASK  = 0x01,
        ML_SHIFT = 4,
        MH_MASK  = 0x01,
        MH_SHIFT = 3 
    };

};
