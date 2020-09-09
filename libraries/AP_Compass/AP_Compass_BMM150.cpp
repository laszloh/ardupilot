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
#include "AP_Compass_BMM150.h"

#include <AP_HAL/AP_HAL.h>

#include <assert.h>
#include <utility>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor_BMI160.h>

#include <stdio.h>

#define CHIP_ID_REG 0x40
#define CHIP_ID_VAL 0x32

#define POWER_AND_OPERATIONS_REG 0x4B
#define POWER_CONTROL_VAL (1 << 0)
#define SOFT_RESET (1 << 7 | 1 << 1)

#define OP_MODE_SELF_TEST_ODR_REG 0x4C
#define NORMAL_MODE (0 << 1)
#define ODR_30HZ (1 << 3 | 1 << 4 | 1 << 5)
#define ODR_20HZ (1 << 3 | 0 << 4 | 1 << 5)

#define DATA_X_LSB_REG 0x42

#define REPETITIONS_XY_REG 0x51
#define REPETITIONS_Z_REG 0X52

/* Trim registers */
#define DIG_X1_REG 0x5D
#define DIG_Y1_REG 0x5E
#define DIG_Z4_LSB_REG 0x62
#define DIG_Z4_MSB_REG 0x63
#define DIG_X2_REG 0x64
#define DIG_Y2_REG 0x65
#define DIG_Z2_LSB_REG 0x68
#define DIG_Z2_MSB_REG 0x69
#define DIG_Z1_LSB_REG 0x6A
#define DIG_Z1_MSB_REG 0x6B
#define DIG_XYZ1_LSB_REG 0x6C
#define DIG_XYZ1_MSB_REG 0x6D
#define DIG_Z3_LSB_REG 0x6E
#define DIG_Z3_MSB_REG 0x6F
#define DIG_XY2_REG 0x70
#define DIG_XY1_REG 0x71

#define MEASURE_TIME_USEC 16667

#define BMM150_DEFAULT_I2C_ADDRESS 0x10

extern const AP_HAL::HAL &hal;

static le16_t sample_reg[4];

AP_Compass_Backend *AP_Compass_BMM150::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_BMM150_BusDriver *bus = new AP_BMM150_BusDriver_HALDevice(std::move(dev));
    if (!bus) {
        return nullptr;
    }

    AP_Compass_BMM150 *sensor = new AP_Compass_BMM150(bus, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/* Probe for BMM150 on auxiliary bus of BMI160, connected through I2C */
AP_Compass_Backend *AP_Compass_BMM150::probe_bmi160(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                    enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor &ins = *AP_InertialSensor::get_singleton();
    ins.detect_backends();

    return probe(std::move(dev), rotation);
}

/* Probe for BMM150 on auxiliary bus of BMI160, connected through SPI */
AP_Compass_Backend *AP_Compass_BMM150::probe_bmi160(uint8_t bmi160_instance,
                                                    enum Rotation rotation)
{
    AP_InertialSensor &ins = *AP_InertialSensor::get_singleton();

    AP_BMM150_BusDriver *bus =
        new AP_BMM150_BusDriver_Auxiliary(ins, HAL_INS_MPU9250_SPI, bmi160_instance, BMM150_DEFAULT_I2C_ADDRESS);
    if (!bus) {
        return nullptr;
    }

    AP_Compass_BMM150 *sensor = new AP_Compass_BMM150(bus, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_BMM150::AP_Compass_BMM150(AP_BMM150_BusDriver *bus, enum Rotation rotation)
    : _bus(bus), _rotation(rotation)
{
}

AP_Compass_BMM150::~AP_Compass_BMM150()
{
    delete _bus;
}

bool AP_Compass_BMM150::_load_trim_values()
{
    struct {
        int8_t dig_x1;
        int8_t dig_y1;
        uint8_t rsv[3];
        le16_t dig_z4;
        int8_t dig_x2;
        int8_t dig_y2;
        uint8_t rsv2[2];
        le16_t dig_z2;
        le16_t dig_z1;
        le16_t dig_xyz1;
        le16_t dig_z3;
        int8_t dig_xy2;
        uint8_t dig_xy1;
    } PACKED trim_registers, trim_registers2;

    // read the registers twice to confirm we have the right
    // values. There is no CRC in the registers and these values are
    // vital to correct operation
    int8_t tries = 4;
    while (tries--) {
        if (!_bus->block_read(DIG_X1_REG, (uint8_t *)&trim_registers,
                                  sizeof(trim_registers))) {
            continue;
        }
        if (!_bus->block_read(DIG_X1_REG, (uint8_t *)&trim_registers2,
                                  sizeof(trim_registers))) {
            continue;
        }
        if (memcmp(&trim_registers, &trim_registers2, sizeof(trim_registers)) == 0) {
            break;
        }
    }
    if (-1 == tries) {
        hal.console->printf("BMM150: Failed to load trim registers\n");
        return false;
    }

    _dig.x1 = trim_registers.dig_x1;
    _dig.x2 = trim_registers.dig_x2;
    _dig.xy1 = trim_registers.dig_xy1;
    _dig.xy2 = trim_registers.dig_xy2;
    _dig.xyz1 = le16toh(trim_registers.dig_xyz1);
    _dig.y1 = trim_registers.dig_y1;
    _dig.y2 = trim_registers.dig_y2;
    _dig.z1 = le16toh(trim_registers.dig_z1);
    _dig.z2 = le16toh(trim_registers.dig_z2);
    _dig.z3 = le16toh(trim_registers.dig_z3);
    _dig.z4 = le16toh(trim_registers.dig_z4);

    return true;
}

bool AP_Compass_BMM150::init()
{
    AP_HAL::Semaphore *bus_sem = _bus->get_semaphore();

    if (!bus_sem) {
        return false;
    }
    bus_sem->take_blocking();

    if(!_bus->configure()) {
        hal.console->printf("BMM150: Could not configure the bus\n");
        goto bus_error;
    }

    if(_check_id()) {
        hal.console->printf("BMM150: Wrong id\n");
        goto bus_error;
    }

    if (!_load_trim_values()) {
        hal.console->printf("BMM150: Could not read calibration data\n");
        goto bus_error;
    }

    if (!_setup_mode()) {
        hal.console->printf("BMM150: Could not setup mode\n");
        goto bus_error;
    }

    if (!_bus->start_measurements()) {
        hal.console->printf("BMM150: Could not start measurements\n");
        goto bus_error;
    }

    _initialized = true;

    /* register the compass instance in the frontend */
    _bus->set_device_type(DEVTYPE_BMM150);
    if (!register_compass(_bus->get_bus_id(), _compass_instance)) {
        goto bus_error;
    }
    set_dev_id(_compass_instance, _bus->get_bus_id());

    set_rotation(_compass_instance, _rotation);
    bus_sem->give();

    _perf_err = hal.util->perf_alloc(AP_HAL::Util::PC_COUNT, "BMM150_err");

    _bus->register_periodic_callback(MEASURE_TIME_USEC, FUNCTOR_BIND_MEMBER(&AP_Compass_BMM150::_update, void));

    _last_read_ms = AP_HAL::millis();

    return true;

bus_error:
    bus_sem->give();
    return false;
}

bool AP_Compass_BMM150::_reset() {
    _bus->register_write(POWER_AND_OPERATIONS_REG, SOFT_RESET);
    hal.scheduler->delay(2);
    return _bus->register_write(POWER_AND_OPERATIONS_REG, POWER_CONTROL_VAL);
}

bool AP_Compass_BMM150::_setup_mode() {
   /*
     * Recommended preset for high accuracy:
     * - Rep X/Y = 47
     * - Rep Z = 83
     * - ODR = 20
     * But we are going to use 30Hz of ODR
     */
    if(!_bus->register_write(REPETITIONS_XY_REG, (47 - 1) / 2))
        return false;
    
    if(!_bus->register_write(REPETITIONS_Z_REG, 83 - 1))
        return false;

    if(!_bus->register_write(OP_MODE_SELF_TEST_ODR_REG, NORMAL_MODE | ODR_30HZ))
        return false;

    return true;
}

bool AP_Compass_BMM150::_check_id() {
    int8_t boot_tries = 4;
    bool ret;
    uint8_t val;

    while (boot_tries--) {
        /* Do a soft reset */
        _reset();

        ret = _bus->register_read(CHIP_ID_REG, &val);
        if (!ret) {
            continue;
        }
        if (val == CHIP_ID_VAL) {
            // found it
            return true;
        }
        if (boot_tries == 0) {
            hal.console->printf("BMM150: Wrong chip ID 0x%02x should be 0x%02x\n", val, CHIP_ID_VAL);
        }
    }
    return false;
}

/*
 * Compensation algorithm got from https://github.com/BoschSensortec/BMM050_driver
 * this is not explained in datasheet.
 */
int16_t AP_Compass_BMM150::_compensate_xy(int16_t xy, uint32_t rhall, int32_t txy1, int32_t txy2)
{
    int32_t inter = ((int32_t)_dig.xyz1) << 14;
    inter /= rhall;
    inter -= 0x4000;

    int32_t val = _dig.xy2 * ((inter * inter) >> 7);
    val += (inter * (((uint32_t)_dig.xy1) << 7));
    val >>= 9;
    val += 0x100000;
    val *= (txy2 + 0xA0);
    val >>= 12;
    val *= xy;
    val >>= 13;
    val += (txy1 << 3);

    return val;
}

int16_t AP_Compass_BMM150::_compensate_z(int16_t z, uint32_t rhall)
{
    int32_t dividend = int32_t(z - _dig.z4) << 15;
    int32_t dividend2 = dividend - ((_dig.z3 * (int32_t(rhall) - int32_t(_dig.xyz1))) >> 2);

    int32_t divisor = int32_t(_dig.z1) * (rhall << 1);
    divisor += 0x8000;
    divisor >>= 16;
    divisor += _dig.z2;

    int16_t ret = constrain_int32(dividend2 / divisor, -0x8000, 0x8000);
#if 0
    static uint8_t counter;
    if (counter++ == 0) {
        printf("ret=%d z=%d rhall=%u z1=%d z2=%d z3=%d z4=%d xyz1=%d dividend=%d dividend2=%d divisor=%d\n",
               ret, z, rhall, _dig.z1, _dig.z2, _dig.z3, _dig.z4, _dig.xyz1, dividend, dividend2, divisor);
    }
#endif
    return ret;
}

void AP_Compass_BMM150::_update()
{
    if(!_bus->block_read(DATA_X_LSB_REG, (uint8_t *) &sample_reg, sizeof(sample_reg)))
        return;

    /* Checking data ready status */
    if (!(sample_reg[3] & 0x1))
        return;

    const uint16_t rhall = le16toh(sample_reg[3]) >> 2;

    Vector3f raw_field = Vector3f{
        (float) _compensate_xy(((int16_t)le16toh(sample_reg[0])) >> 3,
                               rhall, _dig.x1, _dig.x2),
        (float) _compensate_xy(((int16_t)le16toh(sample_reg[1])) >> 3,
                               rhall, _dig.y1, _dig.y2),
        (float) _compensate_z(((int16_t)le16toh(sample_reg[2])) >> 1, rhall)};

    /* apply sensitivity scale 16 LSB/uT */
    raw_field /= 16;
    /* convert uT to milligauss */
    raw_field *= 10;

    _last_read_ms = AP_HAL::millis();

    accumulate_sample(raw_field, _compass_instance);
}

void AP_Compass_BMM150::read()
{
    drain_accumulated_samples(_compass_instance);
}


/* AP_HAL::I2CDevice implementation of the BMM150 */
AP_BMM150_BusDriver_HALDevice::AP_BMM150_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : _dev(std::move(dev))
{
}

bool AP_BMM150_BusDriver_HALDevice::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

bool AP_BMM150_BusDriver_HALDevice::register_read(uint8_t reg, uint8_t *val)
{
    return _dev->read_registers(reg, val, 1);
}

bool AP_BMM150_BusDriver_HALDevice::register_write(uint8_t reg, uint8_t val)
{
    return _dev->write_register(reg, val);
}

AP_HAL::Semaphore *AP_BMM150_BusDriver_HALDevice::get_semaphore()
{
    return _dev->get_semaphore();
}

AP_HAL::Device::PeriodicHandle AP_BMM150_BusDriver_HALDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _dev->register_periodic_callback(period_usec, cb);
}


/* AK8963 on an auxiliary bus of IMU driver */
AP_BMM150_BusDriver_Auxiliary::AP_BMM150_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                                             uint8_t backend_instance, uint8_t addr)
{
    /*
     * Only initialize members. Fails are handled by configure or while
     * getting the semaphore
     */
    _bus = ins.get_auxiliary_bus(backend_id, backend_instance);
    if (!_bus) {
        return;
    }

    _slave = _bus->request_next_slave(addr);
}

AP_BMM150_BusDriver_Auxiliary::~AP_BMM150_BusDriver_Auxiliary()
{
    /* After started it's owned by AuxiliaryBus */
    if (!_started) {
        delete _slave;
    }
}

bool AP_BMM150_BusDriver_Auxiliary::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    if (_started) {
        /*
         * We can only read a block when reading the block of sample values -
         * calling with any other value is a mistake
         */
        assert(reg == DATA_X_LSB_REG);

        int n = _slave->read(buf);
        return n == static_cast<int>(size);
    }

    int r = _slave->passthrough_read(reg, buf, size);

    return r > 0 && static_cast<uint32_t>(r) == size;
}

bool AP_BMM150_BusDriver_Auxiliary::register_read(uint8_t reg, uint8_t *val)
{
    return _slave->passthrough_read(reg, val, 1) == 1;
}

bool AP_BMM150_BusDriver_Auxiliary::register_write(uint8_t reg, uint8_t val)
{
    return _slave->passthrough_write(reg, val) == 1;
}

AP_HAL::Semaphore *AP_BMM150_BusDriver_Auxiliary::get_semaphore()
{
    return _bus ? _bus->get_semaphore() : nullptr;
}

bool AP_BMM150_BusDriver_Auxiliary::configure()
{
    if (!_bus || !_slave) {
        return false;
    }
    return true;
}

bool AP_BMM150_BusDriver_Auxiliary::start_measurements()
{
    if (_bus->register_periodic_read(_slave, DATA_X_LSB_REG, sizeof(sample_reg)) < 0) {
        return false;
    }

    _started = true;

    return true;
}

AP_HAL::Device::PeriodicHandle AP_BMM150_BusDriver_Auxiliary::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _bus->register_periodic_callback(period_usec, cb);
}

// set device type within a device class
void AP_BMM150_BusDriver_Auxiliary::set_device_type(uint8_t devtype)
{
    _bus->set_device_type(devtype);
}

// return 24 bit bus identifier
uint32_t AP_BMM150_BusDriver_Auxiliary::get_bus_id(void) const
{
    return _bus->get_bus_id();
}
