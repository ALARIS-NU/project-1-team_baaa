#pragma once

namespace imu {

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

class MadgwickFilter {
public:
    MadgwickFilter(double sample_period_s = 1.0 / 100.0, double beta = 0.10);

    void setSamplePeriod(double sample_period_s);
    void setBeta(double beta);
    void reset(const Quaternion& q = Quaternion{});

    void update(
        double gx, double gy, double gz,
        double ax, double ay, double az,
        double mx, double my, double mz);

    void updateImu(
        double gx, double gy, double gz,
        double ax, double ay, double az);

    Quaternion quaternion() const;

private:
    static double invSqrt(double value);

    double sample_period_s_;
    double beta_;
    Quaternion q_;
};

}  // namespace imu
