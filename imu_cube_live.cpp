#include "imu_hal.hpp"
#include "madgwick_filter.hpp"

#if __has_include(<GL/freeglut.h>)
#include <GL/freeglut.h>
#else
#include <GL/glut.h>
#endif

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

struct Options {
    std::string device = "/dev/i2c-1";
    double hz = 100.0;
    double beta = 0.10;
    bool simulate = false;
};

std::atomic<bool> g_running{true};
std::mutex g_mutex;
imu::Quaternion g_quat{};
std::string g_status = "Initializing...";

constexpr double kPi = 3.14159265358979323846;

std::array<float, 16> quatToMat4(const imu::Quaternion& q_in) {
    const double n = std::sqrt(q_in.w * q_in.w + q_in.x * q_in.x + q_in.y * q_in.y + q_in.z * q_in.z);
    const imu::Quaternion q = (n > 0.0)
                                  ? imu::Quaternion{q_in.w / n, q_in.x / n, q_in.y / n, q_in.z / n}
                                  : imu::Quaternion{};

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
    m[1] = static_cast<float>(2.0 * (xy + wz));
    m[2] = static_cast<float>(2.0 * (xz - wy));
    m[3] = 0.0f;

    m[4] = static_cast<float>(2.0 * (xy - wz));
    m[5] = static_cast<float>(1.0 - 2.0 * (xx + zz));
    m[6] = static_cast<float>(2.0 * (yz + wx));
    m[7] = 0.0f;

    m[8] = static_cast<float>(2.0 * (xz + wy));
    m[9] = static_cast<float>(2.0 * (yz - wx));
    m[10] = static_cast<float>(1.0 - 2.0 * (xx + yy));
    m[11] = 0.0f;

    m[12] = 0.0f;
    m[13] = 0.0f;
    m[14] = 0.0f;
    m[15] = 1.0f;
    return m;
}

imu::Quaternion quatMul(const imu::Quaternion& a, const imu::Quaternion& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

imu::Quaternion axisAngle(double ax, double ay, double az, double rad) {
    const double n = std::sqrt(ax * ax + ay * ay + az * az);
    if (n <= 1e-12) {
        return {};
    }
    ax /= n;
    ay /= n;
    az /= n;
    const double half = rad * 0.5;
    const double s = std::sin(half);
    return {std::cos(half), ax * s, ay * s, az * s};
}

imu::Quaternion simulatedOrientation(double t_s) {
    const imu::Quaternion q_yaw = axisAngle(0.0, 1.0, 0.0, 0.9 * std::sin(0.6 * t_s));
    const imu::Quaternion q_pitch = axisAngle(1.0, 0.0, 0.0, 0.5 * std::sin(0.8 * t_s));
    const imu::Quaternion q_roll = axisAngle(0.0, 0.0, 1.0, 0.4 * std::cos(0.7 * t_s));
    return quatMul(q_yaw, quatMul(q_pitch, q_roll));
}

std::string hexByte(uint8_t value) {
    std::ostringstream oss;
    oss << "0x" << std::uppercase << std::hex << static_cast<int>(value);
    return oss.str();
}

void drawAxes(float len) {
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
    imu::Quaternion q{};
    std::string status;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        q = g_quat;
        status = g_status;
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 3.2, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    drawAxes(1.25f);

    glPushMatrix();
    const auto m = quatToMat4(q);
    glMultMatrixf(m.data());

    drawAxes(0.9f);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    glColor3f(0.76f, 0.78f, 0.88f);
    glutSolidCube(1.0f);
    glDisable(GL_POLYGON_OFFSET_FILL);

    glColor3f(1.0f, 1.0f, 1.0f);
    glutWireCube(1.01f);
    glPopMatrix();

    drawText2D(0.02f, 0.96f, status);

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

void idle() {
    glutPostRedisplay();
}

void requestExit() {
    g_running.store(false);
#ifdef FREEGLUT
    glutLeaveMainLoop();
#else
    std::exit(0);
#endif
}

void keyboard(unsigned char key, int, int) {
    if ((key == 27) || (key == 'q')) {
        requestExit();
    }
}

Options parseArgs(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            std::cerr
                << "Usage: " << argv[0] << " [options]\n"
                << "Options:\n"
                << "  --device <path>  I2C device path (default /dev/i2c-1)\n"
                << "  --hz <value>     Update frequency (default 100)\n"
                << "  --beta <value>   Madgwick beta gain (default 0.10)\n"
                << "  --simulate       Use synthetic motion (no IMU required)\n";
            std::exit(0);
        } else if ((arg == "--device") && (i + 1 < argc)) {
            options.device = argv[++i];
        } else if ((arg == "--hz") && (i + 1 < argc)) {
            options.hz = std::stod(argv[++i]);
        } else if ((arg == "--beta") && (i + 1 < argc)) {
            options.beta = std::stod(argv[++i]);
        } else if (arg == "--simulate") {
            options.simulate = true;
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    if (options.hz <= 0.0) {
        throw std::runtime_error("--hz must be > 0");
    }
    return options;
}

void sensorLoop(const Options options) {
    try {
        if (options.simulate) {
            const auto start = std::chrono::steady_clock::now();
            while (g_running.load()) {
                const double t_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                const auto q = simulatedOrientation(t_s);
                {
                    std::lock_guard<std::mutex> lock(g_mutex);
                    g_quat = q;
                    g_status = "Simulation mode (no IMU)";
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return;
        }

        imu::LinuxI2CBus bus(options.device);
        imu::BerryImuV3 imu(bus);
        const auto who_lsm6 = imu.whoAmILsm6();
        const auto who_lis3 = imu.whoAmILis3();
        imu.initialize();

        imu::MadgwickFilter filter(1.0 / options.hz, options.beta);
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_status = "IMU active: LSM6 " + hexByte(who_lsm6) +
                       ", LIS3 " + hexByte(who_lis3);
        }

        auto next_tick = std::chrono::steady_clock::now();
        while (g_running.load()) {
            const imu::ImuSample sample = imu.readSample();
            filter.update(
                sample.gyro_rad_s.x, sample.gyro_rad_s.y, sample.gyro_rad_s.z,
                sample.accel_g.x, sample.accel_g.y, sample.accel_g.z,
                sample.mag_gauss.x, sample.mag_gauss.y, sample.mag_gauss.z);

            {
                std::lock_guard<std::mutex> lock(g_mutex);
                g_quat = filter.quaternion();
            }

            next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / options.hz));
            std::this_thread::sleep_until(next_tick);
        }
    } catch (const std::exception& ex) {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_status = std::string("Sensor thread error: ") + ex.what();
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseArgs(argc, argv);

        std::thread sensor_thread(sensorLoop, options);

        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        glutInitWindowSize(960, 640);
        glutCreateWindow("IMU Live Quaternion Cube");

#ifdef GLUT_ACTION_ON_WINDOW_CLOSE
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif

        glEnable(GL_DEPTH_TEST);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);

        glutDisplayFunc(display);
        glutReshapeFunc(reshape);
        glutIdleFunc(idle);
        glutKeyboardFunc(keyboard);
        glutMainLoop();

        g_running.store(false);
        if (sensor_thread.joinable()) {
            sensor_thread.join();
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
