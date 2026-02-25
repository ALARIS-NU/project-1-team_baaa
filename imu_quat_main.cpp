#include "imu_hal.hpp"
#include "madgwick_filter.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

struct Options {
    std::string device = "/dev/i2c-1";
    double hz = 100.0;
    double beta = 0.10;
    int samples = -1;
    bool print_raw = false;
};

std::atomic<bool> g_running{true};

void onSigInt(int) {
    g_running.store(false);
}

[[noreturn]] void printUsageAndExit(const char* argv0, int exit_code) {
    std::cerr
        << "Usage: " << argv0 << " [options]\n"
        << "Options:\n"
        << "  --device <path>     I2C device path (default /dev/i2c-1)\n"
        << "  --hz <value>        Filter/sample frequency in Hz (default 100)\n"
        << "  --beta <value>      Madgwick beta gain (default 0.10)\n"
        << "  --samples <count>   Number of samples to print, -1 for infinite\n"
        << "  --print-raw         Print converted sensor values with quaternion\n"
        << "  --help              Show this help\n";
    std::exit(exit_code);
}

Options parseArgs(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            printUsageAndExit(argv[0], 0);
        } else if (arg == "--print-raw") {
            options.print_raw = true;
        } else if ((arg == "--device") && (i + 1 < argc)) {
            options.device = argv[++i];
        } else if ((arg == "--hz") && (i + 1 < argc)) {
            options.hz = std::stod(argv[++i]);
        } else if ((arg == "--beta") && (i + 1 < argc)) {
            options.beta = std::stod(argv[++i]);
        } else if ((arg == "--samples") && (i + 1 < argc)) {
            options.samples = std::stoi(argv[++i]);
        } else {
            printUsageAndExit(argv[0], 1);
        }
    }
    if (options.hz <= 0.0) {
        throw std::runtime_error("--hz must be > 0");
    }
    if (options.beta < 0.0) {
        throw std::runtime_error("--beta must be >= 0");
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseArgs(argc, argv);

        std::signal(SIGINT, onSigInt);

        imu::LinuxI2CBus bus(options.device);
        imu::BerryImuV3 imu(bus);

        const auto who_lsm6 = imu.whoAmILsm6();
        const auto who_lis3 = imu.whoAmILis3();
        std::cout << "LSM6DS33 WHO_AM_I: 0x" << std::hex << static_cast<int>(who_lsm6) << std::dec << "\n";
        std::cout << "LIS3MDL  WHO_AM_I: 0x" << std::hex << static_cast<int>(who_lis3) << std::dec << "\n";

        imu.initialize();

        imu::MadgwickFilter filter(1.0 / options.hz, options.beta);

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Streaming orientation quaternions. Press Ctrl+C to stop.\n";

        int count = 0;
        auto next_tick = std::chrono::steady_clock::now();
        while (g_running.load()) {
            const imu::ImuSample sample = imu.readSample();
            filter.update(
                sample.gyro_rad_s.x, sample.gyro_rad_s.y, sample.gyro_rad_s.z,
                sample.accel_g.x, sample.accel_g.y, sample.accel_g.z,
                sample.mag_gauss.x, sample.mag_gauss.y, sample.mag_gauss.z);

            const imu::Quaternion q = filter.quaternion();
            if (options.print_raw) {
                std::cout << "G(rad/s)=[" << sample.gyro_rad_s.x << " " << sample.gyro_rad_s.y << " " << sample.gyro_rad_s.z
                          << "] A(g)=[" << sample.accel_g.x << " " << sample.accel_g.y << " " << sample.accel_g.z
                          << "] M(G)=[" << sample.mag_gauss.x << " " << sample.mag_gauss.y << " " << sample.mag_gauss.z
                          << "] q=[" << q.w << " " << q.x << " " << q.y << " " << q.z << "]\n";
            } else {
                std::cout << "q = [w x y z] = " << q.w << " " << q.x << " " << q.y << " " << q.z << "\n";
            }

            ++count;
            if ((options.samples >= 0) && (count >= options.samples)) {
                break;
            }

            next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / options.hz));
            std::this_thread::sleep_until(next_tick);
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
