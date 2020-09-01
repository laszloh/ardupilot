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

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_BMI160_AuxiliaryBus;
class AP_BMI160_AuxiliaryBusSlave;

class AP_InertialSensor_BMI160 : public AP_InertialSensor_Backend {
    friend AP_BMI160_AuxiliaryBus;
    friend AP_BMI160_AuxiliaryBusSlave;

public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;

    bool update() override;

    /*
     * Return an AuxiliaryBus if the bus driver allows it
     */
    AuxiliaryBus *get_auxiliary_bus() override;

    // get a startup banner to output to the GCS
    bool get_output_banner(char* banner, uint8_t banner_len) override;

private:
    AP_InertialSensor_BMI160(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * If the macro BMI160_DEBUG is defined, check if there are errors reported
     * on the device's error register and panic if so. The implementation is
     * empty if the BMI160_DEBUG isn't defined.
     */
    void _check_err_reg();

    /**
     * Try to perform initialization of the BMI160 device.
     *
     * The device semaphore must be taken and released by the caller.
     *
     * @return true on success, false otherwise.
     */
    bool _hardware_init();

    /**
     * Try to initialize this driver.
     *
     * Do sensor and other required initializations.
     *
     * @return true on success, false otherwise.
     */
    bool _init();

    /**
     * Configure accelerometer sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_accel();

    /**
     * Configure gyroscope sensor. The device semaphore must already be
     * taken before calling this function.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_gyro();

    /**
     * Configure INT1 pin as watermark interrupt pin at the level of one sample
     * if using fifo or data-ready pin otherwise.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_int1_pin();

    /**
     * Configure FIFO.
     *
     * @return true on success, false otherwise.
     */
    bool _configure_fifo();

    /**
     * Device periodic callback to read data from the sensors.
     */
    void _poll_data();

    /**
     * Read samples from fifo.
     */
    void _read_fifo();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_BMI160_AuxiliaryBusSlave *_auxiliary_bus;

    uint8_t _accel_instance;
    float _accel_scale;

    uint8_t _gyro_instance;
    float _gyro_scale;

    AP_HAL::DigitalSource *_int1_pin;
};

class AP_BMI160_AuxiliaryBusSlave : public AuxiliaryBusSlave
{
    friend class AP_BMI160_AuxiliaryBus;

public:
    int passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size) override;
    int passthrough_write(uint8_t reg, uint8_t val) override;

    int read(uint8_t *buf) override;

protected:
    AP_BMI160_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr, uint8_t instance);
    int _set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = nullptr);

private:
    const uint8_t _mpu_addr;
    const uint8_t _mpu_reg;
    const uint8_t _mpu_ctrl;
    const uint8_t _mpu_do;

    uint8_t _ext_sens_data = 0;
};

class AP_BMI160_AuxiliaryBus : public AuxiliaryBus
{
    friend class AP_BMI160_Invensense;

public:
    AP_HAL::Semaphore *get_semaphore() override;
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;

protected:
    AP_BMI160_AuxiliaryBus(AP_InertialSensor_Invensense &backend, uint32_t devid);

    AuxiliaryBusSlave *_instantiate_slave(uint8_t addr, uint8_t instance) override;
    int _configure_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg,
                                 uint8_t size) override;

private:
    void _configure_slaves();

    static const uint8_t MAX_EXT_SENS_DATA = 24;
    uint8_t _ext_sens_data = 0;
};