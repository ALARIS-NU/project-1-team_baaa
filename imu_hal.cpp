#include "imu_hal.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>

namespace imu {

namespace {

constexpr double kPi = 3.14159265358979323846;

inline std::runtime_error ioError(const std::string& message) {
    return std::runtime_error(message + ": " + std::strerror(errno));
}

std::string hexByte(uint8_t value) {
    std::ostringstream oss;
    oss << "0x" << std::uppercase << std::hex << static_cast<int>(value);
    return oss.str();
}

}  // namespace

LinuxI2CBus::LinuxI2CBus(const std::string& device_path) {
    fd_ = ::open(device_path.c_str(), O_RDWR);
    if (fd_ < 0) {
        throw ioError("Failed to open I2C device " + device_path);
    }
}

LinuxI2CBus::~LinuxI2CBus() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

void LinuxI2CBus::selectSlave(uint8_t addr) {
    if (addr == active_addr_) {
        return;
    }
    if (ioctl(fd_, I2C_SLAVE, addr) < 0) {
        throw ioError("Failed to select I2C slave " + hexByte(addr));
    }
    active_addr_ = addr;
}

void LinuxI2CBus::writeByte(uint8_t addr, uint8_t reg, uint8_t value) {
    selectSlave(addr);
    uint8_t payload[2] = {reg, value};
    if (::write(fd_, payload, sizeof(payload)) != static_cast<ssize_t>(sizeof(payload))) {
        throw ioError("I2C write failed");
    }
}

uint8_t LinuxI2CBus::readByte(uint8_t addr, uint8_t reg) {
    selectSlave(addr);
    if (::write(fd_, &reg, 1) != 1) {
        throw ioError("I2C register-select write failed");
    }
    uint8_t value = 0;
    if (::read(fd_, &value, 1) != 1) {
        throw ioError("I2C read failed");
    }
    return value;
}

Lsm6ds33::Lsm6ds33(II2CBus& bus, uint8_t i2c_addr) : bus_(bus), addr_(i2c_addr) {}

void Lsm6ds33::configure() {
    constexpr uint8_t REG_CTRL1_XL = 0x10;
    constexpr uint8_t REG_CTRL2_G = 0x11;
    constexpr uint8_t REG_CTRL3_C = 0x12;

    // 104 Hz, +/-2g
    bus_.writeByte(addr_, REG_CTRL1_XL, 0x40);
    // 104 Hz, +/-500 dps
    bus_.writeByte(addr_, REG_CTRL2_G, 0x44);
    // Block Data Update + auto-increment
    bus_.writeByte(addr_, REG_CTRL3_C, 0x44);
}

uint8_t Lsm6ds33::whoAmI() {
    constexpr uint8_t REG_WHO_AM_I = 0x0F;
    return bus_.readByte(addr_, REG_WHO_AM_I);
}

int16_t Lsm6ds33::read16LE(uint8_t reg_low) {
    const auto low = static_cast<uint16_t>(bus_.readByte(addr_, reg_low));
    const auto high = static_cast<uint16_t>(bus_.readByte(addr_, static_cast<uint8_t>(reg_low + 1)));
    return static_cast<int16_t>(low | (high << 8));
}

RawVec3 Lsm6ds33::readGyroRaw() {
    constexpr uint8_t REG_OUTX_L_G = 0x22;
    return {
        read16LE(REG_OUTX_L_G),
        read16LE(static_cast<uint8_t>(REG_OUTX_L_G + 2)),
        read16LE(static_cast<uint8_t>(REG_OUTX_L_G + 4)),
    };
}

RawVec3 Lsm6ds33::readAccelRaw() {
    constexpr uint8_t REG_OUTX_L_XL = 0x28;
    return {
        read16LE(REG_OUTX_L_XL),
        read16LE(static_cast<uint8_t>(REG_OUTX_L_XL + 2)),
        read16LE(static_cast<uint8_t>(REG_OUTX_L_XL + 4)),
    };
}

Lis3mdl::Lis3mdl(II2CBus& bus, uint8_t i2c_addr) : bus_(bus), addr_(i2c_addr) {}

void Lis3mdl::configure() {
    constexpr uint8_t REG_CTRL1 = 0x20;
    constexpr uint8_t REG_CTRL2 = 0x21;
    constexpr uint8_t REG_CTRL3 = 0x22;
    constexpr uint8_t REG_CTRL4 = 0x23;
    constexpr uint8_t REG_CTRL5 = 0x24;

    // Ultra-high-performance mode on XY, ODR=80Hz.
    bus_.writeByte(addr_, REG_CTRL1, 0x7C);
    // Full scale = +/-4 gauss.
    bus_.writeByte(addr_, REG_CTRL2, 0x00);
    // Continuous-conversion mode.
    bus_.writeByte(addr_, REG_CTRL3, 0x00);
    // Ultra-high-performance mode on Z.
    bus_.writeByte(addr_, REG_CTRL4, 0x0C);
    // Block data update.
    bus_.writeByte(addr_, REG_CTRL5, 0x40);
}

uint8_t Lis3mdl::whoAmI() {
    constexpr uint8_t REG_WHO_AM_I = 0x0F;
    return bus_.readByte(addr_, REG_WHO_AM_I);
}

int16_t Lis3mdl::read16LE(uint8_t reg_low) {
    const auto low = static_cast<uint16_t>(bus_.readByte(addr_, reg_low));
    const auto high = static_cast<uint16_t>(bus_.readByte(addr_, static_cast<uint8_t>(reg_low + 1)));
    return static_cast<int16_t>(low | (high << 8));
}

RawVec3 Lis3mdl::readMagRaw() {
    constexpr uint8_t REG_OUT_X_L = 0x28;
    return {
        read16LE(REG_OUT_X_L),
        read16LE(static_cast<uint8_t>(REG_OUT_X_L + 2)),
        read16LE(static_cast<uint8_t>(REG_OUT_X_L + 4)),
    };
}

BerryImuV3::BerryImuV3(II2CBus& bus) : lsm6_(bus), lis3_(bus) {}

void BerryImuV3::initialize() {
    lsm6_.configure();
    lis3_.configure();
}

uint8_t BerryImuV3::whoAmILsm6() {
    return lsm6_.whoAmI();
}

uint8_t BerryImuV3::whoAmILis3() {
    return lis3_.whoAmI();
}

ImuSample BerryImuV3::readSample() {
    ImuSample sample{};
    sample.gyro_raw = lsm6_.readGyroRaw();
    sample.accel_raw = lsm6_.readAccelRaw();
    sample.mag_raw = lis3_.readMagRaw();

    sample.gyro_rad_s = {
        gyroRawToRadPerSec(sample.gyro_raw.x),
        gyroRawToRadPerSec(sample.gyro_raw.y),
        gyroRawToRadPerSec(sample.gyro_raw.z),
    };

    sample.accel_g = {
        accelRawToG(sample.accel_raw.x),
        accelRawToG(sample.accel_raw.y),
        accelRawToG(sample.accel_raw.z),
    };

    sample.mag_gauss = {
        magRawToGauss(sample.mag_raw.x),
        magRawToGauss(sample.mag_raw.y),
        magRawToGauss(sample.mag_raw.z),
    };

    return sample;
}

double BerryImuV3::gyroRawToRadPerSec(int16_t raw) {
    constexpr double kLsm6GyroSensDpsPerLsb = 0.0175;  // +/-500 dps
    const double dps = static_cast<double>(raw) * kLsm6GyroSensDpsPerLsb;
    return dps * (kPi / 180.0);
}

double BerryImuV3::accelRawToG(int16_t raw) {
    constexpr double kLsm6AccelSensGPerLsb = 0.000061;  // +/-2g
    return static_cast<double>(raw) * kLsm6AccelSensGPerLsb;
}

double BerryImuV3::magRawToGauss(int16_t raw) {
    constexpr double kLis3SensLsbPerGauss = 6842.0;  // +/-4 gauss
    return static_cast<double>(raw) / kLis3SensLsbPerGauss;
}

}  // namespace imu
