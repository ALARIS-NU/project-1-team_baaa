#if __has_include(<GL/freeglut.h>)
#include <GL/freeglut.h>
#else
#include <GL/glut.h>
#endif

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
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

// ---------- IMU / I2C ----------
constexpr const char* I2C_DEV = "/dev/i2c-1";
constexpr uint8_t ADDR_LSM6 = 0x6B;  // gyro + accel
constexpr uint8_t ADDR_LIS3 = 0x1E;  // magnetometer

constexpr uint8_t REG_WHO_AM_I_LSM6 = 0x0F;
constexpr uint8_t REG_CTRL1_XL = 0x10;
constexpr uint8_t REG_CTRL2_G = 0x11;
constexpr uint8_t REG_OUTX_L_G = 0x22;
constexpr uint8_t REG_OUTX_L_XL = 0x28;

constexpr uint8_t REG_WHO_AM_I_LIS3 = 0x0F;
constexpr uint8_t REG_CTRL_REG1 = 0x20;
constexpr uint8_t REG_CTRL_REG2 = 0x21;
constexpr uint8_t REG_CTRL_REG3 = 0x22;
constexpr uint8_t REG_OUT_X_L = 0x28;

constexpr double GYRO_DPS_PER_LSB = 0.0175;          // +/-500 dps
constexpr double ACCEL_G_PER_LSB = 0.000061;         // +/-2g
constexpr double MAG_GAUSS_PER_LSB = 1.0 / 6842.0;   // +/-4 gauss

int g_fd = -1;
bool g_imu_ok = false;
std::string g_status = "IMU: init";

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

void closeImu() {
    if (g_fd >= 0) {
        ::close(g_fd);
        g_fd = -1;
    }
}

// ---------- Madgwick ----------
struct Quat {
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

    Quat quat() const {
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
    Quat q_{};
};

MadgwickAHRS g_filter(0.10);
Quat g_q{};
std::chrono::steady_clock::time_point g_last_update;
std::chrono::steady_clock::time_point g_last_print;

// ---------- Quaternion utilities ----------
Quat normalize(const Quat& q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n <= 0.0) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

std::array<float, 16> quatToMat4(const Quat& q_in) {
    const Quat q = normalize(q_in);
    const double w = q.w;
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;

    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;
    const double xy = x * y;
    const double xz = x * z;
    const double yz = y * z;
    const double wx = w * x;
    const double wy = w * y;
    const double wz = w * z;

    std::array<float, 16> m{};
    m[0] = static_cast<float>(1.0 - 2.0 * (yy + zz));
    m[4] = static_cast<float>(2.0 * (xy - wz));
    m[8] = static_cast<float>(2.0 * (xz + wy));
    m[12] = 0.0f;

    m[1] = static_cast<float>(2.0 * (xy + wz));
    m[5] = static_cast<float>(1.0 - 2.0 * (xx + zz));
    m[9] = static_cast<float>(2.0 * (yz - wx));
    m[13] = 0.0f;

    m[2] = static_cast<float>(2.0 * (xz - wy));
    m[6] = static_cast<float>(2.0 * (yz + wx));
    m[10] = static_cast<float>(1.0 - 2.0 * (xx + yy));
    m[14] = 0.0f;

    m[3] = 0.0f;
    m[7] = 0.0f;
    m[11] = 0.0f;
    m[15] = 1.0f;
    return m;
}

void drawAxes(float len = 1.2f) {
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(len, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, len, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, len);
    glEnd();
}

void drawText2D(float x, float y, const std::string& text) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor3f(0.9f, 0.9f, 0.9f);
    glRasterPos2f(x, y);
    for (const char c : text) {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    drawAxes();

    const auto M = quatToMat4(g_q);
    glPushMatrix();
    glMultMatrixf(M.data());

    drawAxes(0.9f);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    glColor3f(0.7f, 0.7f, 0.9f);
    glutSolidCube(1.0f);
    glDisable(GL_POLYGON_OFFSET_FILL);

    glColor3f(1.0f, 1.0f, 1.0f);
    glutWireCube(1.01f);
    glPopMatrix();

    drawText2D(0.02f, 0.96f, g_status);
    glutSwapBuffers();
}

void reshape(int w, int h) {
    const int height = (h > 0) ? h : 1;
    glViewport(0, 0, w, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, static_cast<double>(w) / static_cast<double>(height), 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int, int) {
    if ((key == 27) || (key == 'q')) {
        closeImu();
        std::exit(0);
    }
}

void updateFromImu() {
    const int16_t gx_raw = read16_le(g_fd, ADDR_LSM6, REG_OUTX_L_G);
    const int16_t gy_raw = read16_le(g_fd, ADDR_LSM6, static_cast<uint8_t>(REG_OUTX_L_G + 2));
    const int16_t gz_raw = read16_le(g_fd, ADDR_LSM6, static_cast<uint8_t>(REG_OUTX_L_G + 4));

    const int16_t ax_raw = read16_le(g_fd, ADDR_LSM6, REG_OUTX_L_XL);
    const int16_t ay_raw = read16_le(g_fd, ADDR_LSM6, static_cast<uint8_t>(REG_OUTX_L_XL + 2));
    const int16_t az_raw = read16_le(g_fd, ADDR_LSM6, static_cast<uint8_t>(REG_OUTX_L_XL + 4));

    const int16_t mx_raw = read16_le(g_fd, ADDR_LIS3, REG_OUT_X_L);
    const int16_t my_raw = read16_le(g_fd, ADDR_LIS3, static_cast<uint8_t>(REG_OUT_X_L + 2));
    const int16_t mz_raw = read16_le(g_fd, ADDR_LIS3, static_cast<uint8_t>(REG_OUT_X_L + 4));

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
    double dt = std::chrono::duration<double>(now - g_last_update).count();
    g_last_update = now;
    if ((dt <= 0.0) || (dt > 1.0)) {
        dt = 0.05;
    }

    g_filter.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
    g_q = normalize(g_filter.quat());

    if (now - g_last_print > std::chrono::milliseconds(200)) {
        g_last_print = now;
        std::cout << std::fixed << std::setprecision(6)
                  << "q = [w x y z] = "
                  << g_q.w << " " << g_q.x << " " << g_q.y << " " << g_q.z << "\r" << std::flush;
    }
}

void timerTick(int) {
    if (g_imu_ok) {
        updateFromImu();
        glutPostRedisplay();
    }
    glutTimerFunc(16, timerTick, 0);  // ~60 FPS
}

void initImu() {
    g_fd = i2c_open(I2C_DEV);
    const uint8_t who_lsm6 = read8(g_fd, ADDR_LSM6, REG_WHO_AM_I_LSM6);
    const uint8_t who_lis3 = read8(g_fd, ADDR_LIS3, REG_WHO_AM_I_LIS3);

    write8(g_fd, ADDR_LSM6, REG_CTRL1_XL, 0x20);   // accel: 26Hz, +/-2g
    write8(g_fd, ADDR_LSM6, REG_CTRL2_G, 0x24);    // gyro: 26Hz, +/-500 dps
    write8(g_fd, ADDR_LIS3, REG_CTRL_REG1, 0x6C);  // mag config
    write8(g_fd, ADDR_LIS3, REG_CTRL_REG2, 0x00);  // mag +/-4 gauss
    write8(g_fd, ADDR_LIS3, REG_CTRL_REG3, 0x00);  // continuous mode

    g_imu_ok = true;
    g_status = "IMU ready (LSM6=0x" + std::to_string(static_cast<int>(who_lsm6)) +
               ", LIS3=0x" + std::to_string(static_cast<int>(who_lis3)) + ")";
    g_last_update = std::chrono::steady_clock::now();
    g_last_print = g_last_update;

    std::cout << "LSM6DS33 WHO_AM_I = 0x" << std::hex << static_cast<int>(who_lsm6) << "\n";
    std::cout << "LIS3MDL  WHO_AM_I = 0x" << std::hex << static_cast<int>(who_lis3) << "\n";
    std::cout << std::dec << "Running Task 2: cube is driven by Madgwick quaternion.\n";
    std::cout << "Press q or ESC to quit.\n";
}

void initGL() {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
}

}  // namespace

int main(int argc, char** argv) {
    initImu();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(900, 600);
    glutCreateWindow("Task 2: IMU Madgwick Quaternion Cube");

    initGL();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(16, timerTick, 0);

    glutMainLoop();
    closeImu();
    return 0;
}
