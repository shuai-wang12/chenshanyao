#pragma once

#include "type.h"

struct carconfig
{
    double length_;
    double width_;
    double rear_;
    double wheel_base_;
    double segment_length_;
    double move_step_size_;
    double steering_radian_step_size_;
    double steering_radian_;

    int segment_length_discrete_num_;
    int steering_discrete_num_;

    VecXd vehicle_shape_;
    MatXd vehicle_shape_discrete_;

    void setVehicleShape()
    {
        vehicle_shape_.resize(8);
        vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_, width_ / 2);
        vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length_ - rear_, width_ / 2);
        vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length_ - rear_, -width_ / 2);
        vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_, -width_ / 2);

        const double step_size = move_step_size_;
        const auto N_length = static_cast<unsigned int>(length_ / step_size);
        const auto N_width = static_cast<unsigned int>(width_ / step_size);

        // 车辆形状离散化  矩阵记录每个偏移量
        vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);
        const Vec2d edge_0_normalized =
            (vehicle_shape_.block<2, 1>(2, 0) - vehicle_shape_.block<2, 1>(0, 0))
                .normalized();
        for (unsigned int i = 0; i < N_length; ++i)
        {
            vehicle_shape_discrete_.block<2, 1>(0, i + N_length) =
                vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
            vehicle_shape_discrete_.block<2, 1>(0, i) =
                vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
        }

        const Vec2d edge_1_normalized =
            (vehicle_shape_.block<2, 1>(4, 0) - vehicle_shape_.block<2, 1>(2, 0))
                .normalized();
        for (unsigned int i = 0; i < N_width; ++i)
        {
            vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i) =
                vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
            vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width) =
                vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
        }
    }
};