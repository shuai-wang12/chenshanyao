#pragma once

#include "traj/vehicle_param.h"

struct TrajPlannerConfig{
    int nfe=320;
    double tf=16;
    
    //控制部分的权重
    double opti_w_u = 0.5;

    //转向部分的权重
    double opti_w_r_theta = 50.0;

    //其它参数权重
    double opti_w_rw=10.0;

    //优化的迭代次数
    int opti_iter_max=10;

    double opti_w_penality0=1e5;

    double opti_alpha =5;

    double opti_varepsilon_tol=0.5;

    /**
     * maximum iteration count for corridor expansion
     */
    int corridor_max_iter = 1000;

    /**
     * increment limit for corridor expansion
     */
    double corridor_incremental_limit = 5.0;

    //车辆参数
    VehicleParam vehicle;    
};
