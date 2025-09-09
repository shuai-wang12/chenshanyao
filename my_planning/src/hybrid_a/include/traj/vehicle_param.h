#pragma once

#include <math.h>

#include <tuple>

class VehicleParam{
public:
    double front_hang_length= 1.4;

    double wheel_base=2.0;

    double rear_hang_length =1.3;

    double width=2.0;

    double max_velocity=10.0;

    double min_acceleration=-5.0,max_acceleration=5.0;

    double jerk_max=5.0;

    double phi_max=0.26;

    double omega_max=1.0;

    double radius;
    double f2x,r2x;

    VehicleParam(){
        double length=(wheel_base+rear_hang_length+front_hang_length);
        radius=hypot(0.25*length,0.5*width);
        r2x = 0.25 * length - rear_hang_length;
        f2x = 0.75 * length - rear_hang_length;
    }

    template<class T>
    std::tuple<T, T, T, T> GetDiscPositions(const T &x, const T &y, const T &theta) const {
    auto xf = x + f2x * cos(theta);
    auto xr = x + r2x * cos(theta);
    auto yf = y + f2x * sin(theta);
    auto yr = y + r2x * sin(theta);
    return std::make_tuple(xf, yf, xr, yr);
  }
};