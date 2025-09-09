#pragma once

struct HybridAStarParam{
    double tie_breaker_; // 用于启发式函数避免重复的
    double shot_distance_;
    // penalty
    double steering_penalty_;
    double reversing_penalty_;
    double steering_change_penalty_;
};