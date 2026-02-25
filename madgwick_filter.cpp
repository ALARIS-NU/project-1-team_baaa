#include "madgwick_filter.hpp"

#include <algorithm>
#include <cmath>

namespace imu {

MadgwickFilter::MadgwickFilter(double sample_period_s, double beta)
    : sample_period_s_(sample_period_s), beta_(beta) {}

void MadgwickFilter::setSamplePeriod(double sample_period_s) {
    sample_period_s_ = sample_period_s;
}

void MadgwickFilter::setBeta(double beta) {
    beta_ = beta;
}

void MadgwickFilter::reset(const Quaternion& q) {
    q_ = q;
}

double MadgwickFilter::invSqrt(double value) {
    if (value <= 0.0) {
        return 0.0;
    }
    return 1.0 / std::sqrt(value);
}

void MadgwickFilter::update(
    double gx, double gy, double gz,
    double ax, double ay, double az,
    double mx, double my, double mz) {
    double q1 = q_.w;
    double q2 = q_.x;
    double q3 = q_.y;
    double q4 = q_.z;

    if ((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {
        return;
    }

    if ((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
        updateImu(gx, gy, gz, ax, ay, az);
        return;
    }

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
        s1 = 0.0;
        s2 = 0.0;
        s3 = 0.0;
        s4 = 0.0;
    }

    const double q_dot_1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
    const double q_dot_2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
    const double q_dot_3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
    const double q_dot_4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

    q1 += q_dot_1 * sample_period_s_;
    q2 += q_dot_2 * sample_period_s_;
    q3 += q_dot_3 * sample_period_s_;
    q4 += q_dot_4 * sample_period_s_;

    recip_norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    q_.w = q1 * recip_norm;
    q_.x = q2 * recip_norm;
    q_.y = q3 * recip_norm;
    q_.z = q4 * recip_norm;
}

void MadgwickFilter::updateImu(
    double gx, double gy, double gz,
    double ax, double ay, double az) {
    double q1 = q_.w;
    double q2 = q_.x;
    double q3 = q_.y;
    double q4 = q_.z;

    if ((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {
        return;
    }

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
        s1 = 0.0;
        s2 = 0.0;
        s3 = 0.0;
        s4 = 0.0;
    }

    const double q_dot_1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta_ * s1;
    const double q_dot_2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta_ * s2;
    const double q_dot_3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta_ * s3;
    const double q_dot_4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta_ * s4;

    q1 += q_dot_1 * sample_period_s_;
    q2 += q_dot_2 * sample_period_s_;
    q3 += q_dot_3 * sample_period_s_;
    q4 += q_dot_4 * sample_period_s_;

    recip_norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    q_.w = q1 * recip_norm;
    q_.x = q2 * recip_norm;
    q_.y = q3 * recip_norm;
    q_.z = q4 * recip_norm;
}

Quaternion MadgwickFilter::quaternion() const {
    return q_;
}

}  // namespace imu
