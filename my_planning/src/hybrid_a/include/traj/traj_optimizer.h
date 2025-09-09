#pragma once

#include "traj/discretized_traj.h"
#include "traj/traj_planner_config.h"
#include "traj/traj_nlp.h"
#include "environments.h"
#include "math/aabox2d.h"
#include "math/box2d.h"
#include "math/polygon2d.h"

#include <string>

using math::Polygon2d;
using math::AABox2d;
using math::Box2d;

class TrajOptimizer
{
public:
    // TrajOptimizer()=default;
    explicit TrajOptimizer(const TrajPlannerConfig& config,const environment& _env);
    bool Optimize(std::vector<TrajectoryPoint>& coarse, Constraints& constraints,States& result);
    void dataPrint(std::string& name,States& guess);
private:
    void InitialGuess(States& states,const std::vector<TrajectoryPoint>& coarse);
    bool FormulateCorridorConstraints(States &states, Constraints &constraints);
    bool GenerateBox(double &x, double &y, double radius, AABox2d &result) const;

    TrajPlannerConfig config_;
    TrajectoryNLP nlp_;
    VehicleParam vehicle_;
    const environment &env;
};
