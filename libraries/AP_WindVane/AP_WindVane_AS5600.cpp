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
 *       AP_WindVane_AS5600.cpp
 *       Code by Laszlo Hegedüs
 *
 *       datasheet: https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 *
 *       Sensor should be connected to the I2C port
 */

#include "AP_WindVane_AS5600.h"

#include <utility>

extern const AP_HAL::HAL& hal;

AP_WindVane_AS5600::AP_WindVane_AS5600(AP_WindVane &frontend, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    :AP_WindVane_Backend(frontend),
    _dev(std::move(dev))
{
}

AP_WindVane_Backend *AP_WindVane_AS5600::detect(AP_WindVane &frontend,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_WindVane_AS5600 *sensor = new AP_WindVane_AS5600(frontend, std::move(dev));
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_WindVane_AS5600::_init(void) {
    _dev->get_semaphore()->take_blocking();

    // set up the AS5600 (when in I2C mode only)
    uint8_t regAddr = RegisterMap::AS5600_CONF;
    uint32_t data;

    if(!_dev->transfer(&regAddr, sizeof(regAddr), (uint8_t*)&data, sizeof(data))) {
        _dev->get_semaphore()->give();
        return false;
    }
    // set the polling to 5 ms
    data = (data & (RegisterCONF::PM_MASK << RegisterCONF::PM_SHIFT)) | (PowerMode::PM_5MS << RegisterCONF::PM_SHIFT);
    // enable watchdog (reduce power consumption if wane does not move for 1min)
    data |= (1 << RegisterCONF::WD_SHIFT);

    // add the register address as te first byte
    data = (data << 8)  | regAddr;

    if(!_dev->transfer((uint8_t*)&data, 3, nullptr, 0)) {
        _dev->get_semaphore()->give();
        return false;
    }

    // set up start and end degree to compensate for bearing offset
    uint16_t start_angle = static_cast<uint16_t>(linear_interpolate(0, 1024, _frontend._dir_analog_bearing_offset.get(), 0.0f, 360.0f));

    data = (start_angle << 8) | RegisterMap::AS5600_ZPOS;
    if(!_dev->transfer((uint8_t*)&data, 3, nullptr, 0)) {
        _dev->get_semaphore()->give();
        return false;
    }

    data = ( (start_angle-1) << 8) | RegisterMap::AS5600_MPOS;
    if(!_dev->transfer((uint8_t*)&data, 3, nullptr, 0)) {
        _dev->get_semaphore()->give();
        return false;
    }

    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_WindVane_AS5600::_timer, void));

    return true;
}

void AP_WindVane_AS5600::_timer(void) {
    uint16_t ang;

    if(get_reading(ang)) {
        WITH_SEMAPHORE(_sem);
        angle = ang;
        new_value = true;
        last_read_ms = AP_HAL::millis();
    }
}

bool AP_WindVane_AS5600::get_reading(uint16_t &reading_ang) {
    uint8_t addr = RegisterMap::AS5600_ANGLE;
    uint16_t value;

    bool ret = _dev->transfer(&addr, sizeof(addr), (uint8_t*)&value, sizeof(value));
    if(ret) {
        reading_ang = value;
    }

    return ret;
}

void AP_WindVane_AS5600::update_direction() {
    WITH_SEMAPHORE(_sem);
    if(new_value) {
        new_value = false;

        const float wind_dir_deg = linear_interpolate(0.0f, 360.0f, angle, 0, 1024);
        direction_update_frontend(wrap_PI(radians(wind_dir_deg) + AP::ahrs().yaw));
    }
}

