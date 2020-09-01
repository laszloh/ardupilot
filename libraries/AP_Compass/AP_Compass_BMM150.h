/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_BMM150 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, enum Rotation rotation);

    /* Probe for BMM150 on auxiliary bus of BMI160, connected through I2C */
    static AP_Compass_Backend *probe_bmi160(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                             enum Rotation rotation);

    /* Probe for BMM150 on auxiliary bus of BMI160, connected through SPI */
    static AP_Compass_Backend *probe_bmi160(uint8_t bmi160_instance,
                                             enum Rotation rotation);
    void read() override;

    static constexpr const char *name = "BMM150";

    virtual ~AP_Compass_BMM150();

private:
    AP_Compass_BMM150(AP_BMM150_BusDriver *bus, enum Rotation rotation);

    /**
     * Device periodic callback to read data from the sensor.
     */
    bool init();
    void _update();
    bool _load_trim_values();
    int16_t _compensate_xy(int16_t xy, uint32_t rhall, int32_t txy1, int32_t txy2);
    int16_t _compensate_z(int16_t z, uint32_t rhall);

    AP_BMM150_BusDriver *_bus;

    uint8_t _compass_instance;

    struct {
        int8_t x1;
        int8_t y1;
        int8_t x2;
        int8_t y2;
        uint16_t z1;
        int16_t z2;
        int16_t z3;
        int16_t z4;
        uint8_t xy1;
        int8_t xy2;
        uint16_t xyz1;
    } _dig;

    uint32_t _last_read_ms;
    bool _initialized;
    enum Rotation _rotation;
};

class AP_BMM150_BusDriver
{
public:
    virtual ~AP_BMM150_BusDriver() { }

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) = 0;
    virtual bool register_read(uint8_t reg, uint8_t *val) = 0;
    virtual bool register_write(uint8_t reg, uint8_t val) = 0;

    virtual AP_HAL::Semaphore *get_semaphore() = 0;

    virtual bool configure() { return true; }
    virtual bool start_measurements() { return true; }
    virtual AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t, AP_HAL::Device::PeriodicCb) = 0;

    // set device type within a device class
    virtual void set_device_type(uint8_t devtype) = 0;

    // return 24 bit bus identifier
    virtual uint32_t get_bus_id(void) const = 0;
};

class AP_BMM150_BusDriver_HALDevice: public AP_BMM150_BusDriver
{
public:
    AP_BMM150_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    virtual bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    virtual bool register_read(uint8_t reg, uint8_t *val) override;
    virtual bool register_write(uint8_t reg, uint8_t val) override;

    virtual AP_HAL::Semaphore  *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    // set device type within a device class
    void set_device_type(uint8_t devtype) override {
        _dev->set_device_type(devtype);
    }

    // return 24 bit bus identifier
    uint32_t get_bus_id(void) const override {
        return _dev->get_bus_id();
    }
    
private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

class AP_BMM150_BusDriver_Auxiliary : public AP_BMM150_BusDriver
{
public:
    AP_BMM150_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                  uint8_t backend_instance, uint8_t addr);
    ~AP_BMM150_BusDriver_Auxiliary();

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    bool register_read(uint8_t reg, uint8_t *val) override;
    bool register_write(uint8_t reg, uint8_t val) override;

    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;
    
    AP_HAL::Semaphore  *get_semaphore() override;

    bool configure() override;
    bool start_measurements() override;

    // set device type within a device class
    void set_device_type(uint8_t devtype) override;

    // return 24 bit bus identifier
    uint32_t get_bus_id(void) const override;
    
private:
    AuxiliaryBus *_bus;
    AuxiliaryBusSlave *_slave;
    bool _started;
};
