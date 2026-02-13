#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>

static int i2c_open(const char* dev) {
    int fd = ::open(dev, O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open " << dev << ": " << std::strerror(errno) << "\n";
        std::exit(1);
    }
    return fd;
}

static void i2c_set_slave(int fd, uint8_t addr) {
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        std::cerr << "Failed to set I2C slave 0x" << std::hex << int(addr)
                  << ": " << std::strerror(errno) << "\n";
        std::exit(1);
    }
}

static uint8_t read8(int fd, uint8_t addr, uint8_t reg) {
    i2c_set_slave(fd, addr);
    if (write(fd, &reg, 1) != 1) {
        std::cerr << "I2C write(reg) failed: " << std::strerror(errno) << "\n";
        std::exit(1);
    }
    uint8_t v = 0;
    if (read(fd, &v, 1) != 1) {
        std::cerr << "I2C read failed: " << std::strerror(errno) << "\n";
        std::exit(1);
    }
    return v;
}

static void write8(int fd, uint8_t addr, uint8_t reg, uint8_t val) {
    i2c_set_slave(fd, addr);
    uint8_t buf[2] = {reg, val};
    if (write(fd, buf, 2) != 2) {
        std::cerr << "I2C write(reg,val) failed: " << std::strerror(errno) << "\n";
        std::exit(1);
    }
}

static int16_t read16_le(int fd, uint8_t addr, uint8_t reg_l) {
    uint8_t lo = read8(fd, addr, reg_l);
    uint8_t hi = read8(fd, addr, uint8_t(reg_l + 1));
    return int16_t(uint16_t(lo) | (uint16_t(hi) << 8));
}

int main() {
    constexpr const char* I2C_DEV = "/dev/i2c-1";

    constexpr uint8_t ADDR_LSM6 = 0x6B; // gyro+accel
    constexpr uint8_t ADDR_LIS3 = 0x1E; // magnetometer

    // LSM6DS33 registers 
    constexpr uint8_t REG_WHO_AM_I_LSM6 = 0x0F;
    constexpr uint8_t REG_CTRL1_XL      = 0x10;
    constexpr uint8_t REG_CTRL2_G       = 0x11;

    // Output regs
    constexpr uint8_t REG_OUTX_L_G  = 0x22;
    constexpr uint8_t REG_OUTX_L_XL = 0x28;

    // LIS3MDL registers 
    constexpr uint8_t REG_WHO_AM_I_LIS3 = 0x0F;
    constexpr uint8_t REG_CTRL_REG1     = 0x20;
    constexpr uint8_t REG_CTRL_REG2     = 0x21;
    constexpr uint8_t REG_CTRL_REG3     = 0x22;

    constexpr uint8_t REG_OUT_X_L = 0x28;

    int fd = i2c_open(I2C_DEV);

    // WHO_AM_I regs
    uint8_t who_lsm6 = read8(fd, ADDR_LSM6, REG_WHO_AM_I_LSM6);
    uint8_t who_lis3 = read8(fd, ADDR_LIS3, REG_WHO_AM_I_LIS3);

    std::cout << "LSM6DS33 WHO_AM_I = 0x" << std::hex << int(who_lsm6) << "\n";
    std::cout << "LIS3MDL  WHO_AM_I = 0x" << std::hex << int(who_lis3) << "\n\n";

    // Expected from datasheets:
    // - LSM6DS33 fixed 0x69
    // - LIS3MDL  fixed 0x3D

    // 2) Configure LSM6DS33 accel: 26 Hz, ±2g
    // CTRL1_XL: [7:4]=ODR_XL, [3:2]=FS_XL, [1:0]=BW_XL
    // ODR=26Hz -> 0b0010, FS=±2g -> 0b00, BW=400Hz -> 0b00
    // => 0b0010_0000 = 0x20
    write8(fd, ADDR_LSM6, REG_CTRL1_XL, 0x20);

    // 3) Configure LSM6DS33 gyro: 26 Hz, ±500 dps
    // CTRL2_G: [7:4]=ODR_G, [3:2]=FS_G (500 dps -> 01), other bits 0
    // ODR=26Hz -> 0b0010 in [7:4], FS=01 in [3:2]
    // => 0b0010_0100 = 0x24
    write8(fd, ADDR_LSM6, REG_CTRL2_G, 0x24);

    // 4) Configure LIS3MDL mag:
    // CTRL_REG1: TEMP_EN(7)=0, OM(6:5)=11 (UHP), DO(4:2)=011 (5Hz), FAST_ODR(1)=0, ST(0)=0
    // => 0b0110_1100 = 0x6C
    write8(fd, ADDR_LIS3, REG_CTRL_REG1, 0x6C);

    // CTRL_REG2: FS=00 => ±4 gauss
    write8(fd, ADDR_LIS3, REG_CTRL_REG2, 0x00);

    // CTRL_REG3: MD=00 => continuous-conversion mode (not power-down)
    write8(fd, ADDR_LIS3, REG_CTRL_REG3, 0x00);

    std::cout << "Configured sensors. Streaming raw data...\n";
    std::cout << "Press Ctrl+C to stop.\n\n";

    std::cout << std::dec
              << "  Gx     Gy     Gz  |  Ax     Ay     Az  |  Mx     My     Mz\n"
              << "-----------------------------------------------------------------\n";

    while (true) {
        // Gyro raw
        int16_t gx = read16_le(fd, ADDR_LSM6, REG_OUTX_L_G);
        int16_t gy = read16_le(fd, ADDR_LSM6, uint8_t(REG_OUTX_L_G + 2));
        int16_t gz = read16_le(fd, ADDR_LSM6, uint8_t(REG_OUTX_L_G + 4));

        // Accel raw
        int16_t ax = read16_le(fd, ADDR_LSM6, REG_OUTX_L_XL);
        int16_t ay = read16_le(fd, ADDR_LSM6, uint8_t(REG_OUTX_L_XL + 2));
        int16_t az = read16_le(fd, ADDR_LSM6, uint8_t(REG_OUTX_L_XL + 4));

        // Mag raw
        int16_t mx = read16_le(fd, ADDR_LIS3, REG_OUT_X_L);
        int16_t my = read16_le(fd, ADDR_LIS3, uint8_t(REG_OUT_X_L + 2));
        int16_t mz = read16_le(fd, ADDR_LIS3, uint8_t(REG_OUT_X_L + 4));

        std::cout << std::setw(6) << gx << " " << std::setw(6) << gy << " " << std::setw(6) << gz
                  << " | " << std::setw(6) << ax << " " << std::setw(6) << ay << " " << std::setw(6) << az
                  << " | " << std::setw(6) << mx << " " << std::setw(6) << my << " " << std::setw(6) << mz
                  << "\r" << std::flush;

        usleep(50'000); // 50 ms refresh (20 Hz printing)
    }

    close(fd);
    return 0;
}

