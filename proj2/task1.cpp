#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

volatile std::sig_atomic_t g_running = 1;

void onSigInt(int) {
    g_running = 0;
}

int i2c_open(const char* dev) {
    const int fd = ::open(dev, O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open " << dev << ": " << std::strerror(errno) << "\n";
        std::exit(1);
    }
    return fd;
}

void i2c_set_slave(int fd, uint8_t addr) {
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        std::cerr << "Failed to set I2C slave 0x" << std::hex << static_cast<int>(addr)
                  << ": " << std::strerror(errno) << "\n";
        std::exit(1);
    }
}

uint8_t read8(int fd, uint8_t addr, uint8_t reg) {
    i2c_set_slave(fd, addr);
    if (::write(fd, &reg, 1) != 1) {
        std::cerr << "I2C write(reg) failed: " << std::strerror(errno) << "\n";
        std::exit(1);
    }
    uint8_t value = 0;
    if (::read(fd, &value, 1) != 1) {
        std::cerr << "I2C read failed: " << std::strerror(errno) << "\n";
        std::exit(1);
    }
    return value;
}

void write8(int fd, uint8_t addr, uint8_t reg, uint8_t value) {
    i2c_set_slave(fd, addr);
    const uint8_t payload[2] = {reg, value};
    if (::write(fd, payload, 2) != 2) {
        std::cerr << "I2C write(reg,val) failed: " << std::strerror(errno) << "\n";
        std::exit(1);
    }
}

int16_t read16_le(int fd, uint8_t addr, uint8_t reg_l) {
    const uint8_t lo = read8(fd, addr, reg_l);
    const uint8_t hi = read8(fd, addr, static_cast<uint8_t>(reg_l + 1));
    return static_cast<int16_t>(static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8));
}

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

class MadgwickAHRS {
public:
    explicit MadgwickAHRS(double beta = 0.10) : beta_(beta) {}

    void update(
        double gx, double gy, double gz,
        double ax, double ay, double az,
        double mx, double my, double mz,
        double dt_s) {
        if ((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {
            return;
        }
        if ((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
            updateImu(gx, gy, gz, ax, ay, az, dt_s);
            return;
        }

        double q1 = q_.w;
        double q2 = q_.x;
        double q3 = q_.y;
        double q4 = q_.z;

        double recip_norm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        recip_norm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;

        const double _2q1mx = 2.0 * q1 * mx;
        const double _2q1my = 2.0 * q1 * my;
        const double _2q1mz = 2.0 * q1 * mz;
        const double _2q2mx = 2.0 * q2 * mx;
        const double _2q1 = 2.0 * q1;
        const double _2q2 = 2.0 * q2;
        const double _2q3 = 2.0 * q3;
        const double _2q4 = 2.0 * q4;
        const double q1q1 = q1 * q1;
        const double q1q2 = q1 * q2;
        const double q1q3 = q1 * q3;
        const double q1q4 = q1 * q4;
        const double q2q2 = q2 * q2;
        const double q2q3 = q2 * q3;
        const double q2q4 = q2 * q4;
        const double q3q3 = q3 * q3;
        const double q3q4 = q3 * q4;
        const double q4q4 = q4 * q4;

        const double hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 +
                          mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 -
                          mx * q3q3 - mx * q4q4;
        const double hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 +
                          _2q2mx * q3 - my * q2q2 + my * q3q3 +
                          _2q3 * mz * q4 - my * q4q4;
        const double _2bx = std::sqrt(hx * hx + hy * hy);
        const double _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 +
                            _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 -
                            mz * q3q3 + mz * q4q4;
        const double _4bx = 2.0 * _2bx;
        const double _4bz = 2.0 * _2bz;

        double s1 = -_2q3 * (2.0 * (q2q4 - q1q3) - ax) +
                    _2q2 * (2.0 * (q1q2 + q3q4) - ay) -
                    _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) +
                                 _2bz * (q2q4 - q1q3) - mx) +
                    (-_2bx * q4 + _2bz * q2) *
                        (_2bx * (q2q3 - q1q4) +
                         _2bz * (q1q2 + q3q4) - my) +
                    _2bx * q3 * (_2bx * (q1q3 + q2q4) +
                                 _2bz * (0.5 - q2q2 - q3q3) - mz);

        double s2 = _2q4 * (2.0 * (q2q4 - q1q3) - ax) +
                    _2q1 * (2.0 * (q1q2 + q3q4) - ay) -
                    4.0 * q2 * (1.0 - 2.0 * (q2q2 + q3q3) - az) +
                    _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) +
                                 _2bz * (q2q4 - q1q3) - mx) +
                    (_2bx * q3 + _2bz * q1) *
                        (_2bx * (q2q3 - q1q4) +
                         _2bz * (q1q2 + q3q4) - my) +
                    (_2bx * q4 - _4bz * q2) *
                        (_2bx * (q1q3 + q2q4) +
                         _2bz * (0.5 - q2q2 - q3q3) - mz);

        double s3 = -_2q1 * (2.0 * (q2q4 - q1q3) - ax) +
                    _2q4 * (2.0 * (q1q2 + q3q4) - ay) -
                    4.0 * q3 * (1.0 - 2.0 * (q2q2 + q3q3) - az) +
                    (-_4bx * q3 - _2bz * q1) *
                        (_2bx * (0.5 - q3q3 - q4q4) +
                         _2bz * (q2q4 - q1q3) - mx) +
                    (_2bx * q2 + _2bz * q4) *
                        (_2bx * (q2q3 - q1q4) +
                         _2bz * (q1q2 + q3q4) - my) +
                    (_2bx * q1 - _4bz * q3) *
                        (_2bx * (q1q3 + q2q4) +
                         _2bz * (0.5 - q2q2 - q3q3) - mz);

        double s4 = _2q2 * (2.0 * (q2q4 - q1q3) - ax) +
                    _2q3 * (2.0 * (q1q2 + q3q4) - ay) +
                    (-_4bx * q4 + _2bz * q2) *
                        (_2bx * (0.5 - q3q3 - q4q4) +
                         _2bz * (q2q4 - q1q3) - mx) +
                    (-_2bx * q1 + _2bz * q3) *
                        (_2bx * (q2q3 - q1q4) +
                         _2bz * (q1q2 + q3q4) - my) +
                    _2bx * q2 * (_2bx * (q1q3 + q2q4) +
                                 _2bz * (0.5 - q2q2 - q3q3) - mz);

        const double step_norm = s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4;
        if (step_norm > 0.0) {
            recip_norm = invSqrt(step_norm);
            s1 *= recip_norm;
            s2 *= recip_norm;
            s3 *= recip_norm;
            s4 *= recip_norm;
        } else {
            s1 = s2 = s3 = s4 = 0.0;
        }

        const double q_dot_1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
        const double q_dot_2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
        const double q_dot_3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
        const double q_dot_4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

        q1 += q_dot_1 * dt_s;
        q2 += q_dot_2 * dt_s;
        q3 += q_dot_3 * dt_s;
        q4 += q_dot_4 * dt_s;

        recip_norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        q_.w = q1 * recip_norm;
        q_.x = q2 * recip_norm;
        q_.y = q3 * recip_norm;
        q_.z = q4 * recip_norm;
    }

    Quaternion quaternion() const {
        return q_;
    }

private:
    static double invSqrt(double v) {
        if (v <= 0.0) {
            return 0.0;
        }
        return 1.0 / std::sqrt(v);
    }

    void updateImu(
        double gx, double gy, double gz,
        double ax, double ay, double az,
        double dt_s) {
        if ((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {
            return;
        }

        double q1 = q_.w;
        double q2 = q_.x;
        double q3 = q_.y;
        double q4 = q_.z;

        double recip_norm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        const double _2q1 = 2.0 * q1;
        const double _2q2 = 2.0 * q2;
        const double _2q3 = 2.0 * q3;
        const double _2q4 = 2.0 * q4;
        const double _4q1 = 4.0 * q1;
        const double _4q2 = 4.0 * q2;
        const double _4q3 = 4.0 * q3;
        const double _8q2 = 8.0 * q2;
        const double _8q3 = 8.0 * q3;
        const double q1q1 = q1 * q1;
        const double q2q2 = q2 * q2;
        const double q3q3 = q3 * q3;
        const double q4q4 = q4 * q4;

        double s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
        double s2 = _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2 - _2q1 * ay -
                    _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
        double s3 = 4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay -
                    _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
        double s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay;

        const double step_norm = s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4;
        if (step_norm > 0.0) {
            recip_norm = invSqrt(step_norm);
            s1 *= recip_norm;
            s2 *= recip_norm;
            s3 *= recip_norm;
            s4 *= recip_norm;
        } else {
            s1 = s2 = s3 = s4 = 0.0;
        }

        const double q_dot_1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
        const double q_dot_2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
        const double q_dot_3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
        const double q_dot_4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

        q1 += q_dot_1 * dt_s;
        q2 += q_dot_2 * dt_s;
        q3 += q_dot_3 * dt_s;
        q4 += q_dot_4 * dt_s;

        recip_norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        q_.w = q1 * recip_norm;
        q_.x = q2 * recip_norm;
        q_.y = q3 * recip_norm;
        q_.z = q4 * recip_norm;
    }

    double beta_;
    Quaternion q_{};
};

}  // namespace

int main() {
    std::signal(SIGINT, onSigInt);

    constexpr const char* I2C_DEV = "/dev/i2c-1";
    constexpr uint8_t ADDR_LSM6 = 0x6B;  // gyro + accel
    constexpr uint8_t ADDR_LIS3 = 0x1E;  // magnetometer

    // LSM6DS33 registers
    constexpr uint8_t REG_WHO_AM_I_LSM6 = 0x0F;
    constexpr uint8_t REG_CTRL1_XL = 0x10;
    constexpr uint8_t REG_CTRL2_G = 0x11;
    constexpr uint8_t REG_OUTX_L_G = 0x22;
    constexpr uint8_t REG_OUTX_L_XL = 0x28;

    // LIS3MDL registers
    constexpr uint8_t REG_WHO_AM_I_LIS3 = 0x0F;
    constexpr uint8_t REG_CTRL_REG1 = 0x20;
    constexpr uint8_t REG_CTRL_REG2 = 0x21;
    constexpr uint8_t REG_CTRL_REG3 = 0x22;
    constexpr uint8_t REG_OUT_X_L = 0x28;

    // Scaling constants to the units Madgwick expects.
    constexpr double GYRO_DPS_PER_LSB = 0.0175;   // LSM6DS33, +/-500 dps
    constexpr double ACCEL_G_PER_LSB = 0.000061;  // LSM6DS33, +/-2g
    constexpr double MAG_GAUSS_PER_LSB = 1.0 / 6842.0;  // LIS3MDL, +/-4 gauss

    const int fd = i2c_open(I2C_DEV);

    const uint8_t who_lsm6 = read8(fd, ADDR_LSM6, REG_WHO_AM_I_LSM6);
    const uint8_t who_lis3 = read8(fd, ADDR_LIS3, REG_WHO_AM_I_LIS3);
    std::cout << "LSM6DS33 WHO_AM_I = 0x" << std::hex << static_cast<int>(who_lsm6) << "\n";
    std::cout << "LIS3MDL  WHO_AM_I = 0x" << std::hex << static_cast<int>(who_lis3) << "\n\n";

    // Keep the same register setup from code.cpp.
    write8(fd, ADDR_LSM6, REG_CTRL1_XL, 0x20);  // accel: 26Hz, +/-2g
    write8(fd, ADDR_LSM6, REG_CTRL2_G, 0x24);   // gyro: 26Hz, +/-500dps
    write8(fd, ADDR_LIS3, REG_CTRL_REG1, 0x6C); // mag config
    write8(fd, ADDR_LIS3, REG_CTRL_REG2, 0x00); // mag +/-4 gauss
    write8(fd, ADDR_LIS3, REG_CTRL_REG3, 0x00); // continuous mode

    MadgwickAHRS filter(0.10);
    auto last = std::chrono::steady_clock::now();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Streaming quaternion (Task 1). Press Ctrl+C to stop.\n";
    std::cout << "q = [w x y z]\n";

    while (g_running) {
        const int16_t gx_raw = read16_le(fd, ADDR_LSM6, REG_OUTX_L_G);
        const int16_t gy_raw = read16_le(fd, ADDR_LSM6, static_cast<uint8_t>(REG_OUTX_L_G + 2));
        const int16_t gz_raw = read16_le(fd, ADDR_LSM6, static_cast<uint8_t>(REG_OUTX_L_G + 4));

        const int16_t ax_raw = read16_le(fd, ADDR_LSM6, REG_OUTX_L_XL);
        const int16_t ay_raw = read16_le(fd, ADDR_LSM6, static_cast<uint8_t>(REG_OUTX_L_XL + 2));
        const int16_t az_raw = read16_le(fd, ADDR_LSM6, static_cast<uint8_t>(REG_OUTX_L_XL + 4));

        const int16_t mx_raw = read16_le(fd, ADDR_LIS3, REG_OUT_X_L);
        const int16_t my_raw = read16_le(fd, ADDR_LIS3, static_cast<uint8_t>(REG_OUT_X_L + 2));
        const int16_t mz_raw = read16_le(fd, ADDR_LIS3, static_cast<uint8_t>(REG_OUT_X_L + 4));

        // Required scaling:
        // gyro -> rad/s, accel -> g, mag -> consistent unit (gauss here).
        const double gx = static_cast<double>(gx_raw) * GYRO_DPS_PER_LSB * (kPi / 180.0);
        const double gy = static_cast<double>(gy_raw) * GYRO_DPS_PER_LSB * (kPi / 180.0);
        const double gz = static_cast<double>(gz_raw) * GYRO_DPS_PER_LSB * (kPi / 180.0);

        const double ax = static_cast<double>(ax_raw) * ACCEL_G_PER_LSB;
        const double ay = static_cast<double>(ay_raw) * ACCEL_G_PER_LSB;
        const double az = static_cast<double>(az_raw) * ACCEL_G_PER_LSB;

        const double mx = static_cast<double>(mx_raw) * MAG_GAUSS_PER_LSB;
        const double my = static_cast<double>(my_raw) * MAG_GAUSS_PER_LSB;
        const double mz = static_cast<double>(mz_raw) * MAG_GAUSS_PER_LSB;

        const auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;
        if ((dt <= 0.0) || (dt > 1.0)) {
            dt = 0.05;
        }

        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
        const Quaternion q = filter.quaternion();

        std::cout << "q = [" << q.w << " " << q.x << " " << q.y << " " << q.z << "]\r" << std::flush;
        usleep(50'000);  // 20 Hz print/update
    }

    std::cout << "\nStopped.\n";
    ::close(fd);
    return 0;
}
