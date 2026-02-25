#pragma once

#include <cstdint>
#include <string>

namespace imu {

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct RawVec3 {
    int16_t x{0};
    int16_t y{0};
    int16_t z{0};
};

struct ImuSample {
    RawVec3 gyro_raw;
    RawVec3 accel_raw;
    RawVec3 mag_raw;
    Vec3 gyro_rad_s;
    Vec3 accel_g;
    Vec3 mag_gauss;
};

class II2CBus {
public:
    virtual ~II2CBus() = default;
    virtual void writeByte(uint8_t addr, uint8_t reg, uint8_t value) = 0;
    virtual uint8_t readByte(uint8_t addr, uint8_t reg) = 0;
};

class LinuxI2CBus : public II2CBus {
public:
    explicit LinuxI2CBus(const std::string& device_path);
    ~LinuxI2CBus() override;

    LinuxI2CBus(const LinuxI2CBus&) = delete;
    LinuxI2CBus& operator=(const LinuxI2CBus&) = delete;

    void writeByte(uint8_t addr, uint8_t reg, uint8_t value) override;
    uint8_t readByte(uint8_t addr, uint8_t reg) override;

private:
    void selectSlave(uint8_t addr);

    int fd_{-1};
    uint8_t active_addr_{0xFF};
};

class Lsm6ds33 {
public:
    explicit Lsm6ds33(II2CBus& bus, uint8_t i2c_addr = 0x6B);
    void configure();
    uint8_t whoAmI();
    RawVec3 readGyroRaw();
    RawVec3 readAccelRaw();

private:
    int16_t read16LE(uint8_t reg_low);

    II2CBus& bus_;
    uint8_t addr_;
};

class Lis3mdl {
public:
    explicit Lis3mdl(II2CBus& bus, uint8_t i2c_addr = 0x1E);
    void configure();
    uint8_t whoAmI();
    RawVec3 readMagRaw();

private:
    int16_t read16LE(uint8_t reg_low);

    II2CBus& bus_;
    uint8_t addr_;
};

class BerryImuV3 {
public:
    explicit BerryImuV3(II2CBus& bus);
    void initialize();
    uint8_t whoAmILsm6();
    uint8_t whoAmILis3();
    ImuSample readSample();

    static double gyroRawToRadPerSec(int16_t raw);
    static double accelRawToG(int16_t raw);
    static double magRawToGauss(int16_t raw);

private:
    Lsm6ds33 lsm6_;
    Lis3mdl lis3_;
};

}  // namespace imu
