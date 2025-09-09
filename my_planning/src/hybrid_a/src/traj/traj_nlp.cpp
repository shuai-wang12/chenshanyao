#include "traj/traj_nlp.h"

TrajectoryNLP::TrajectoryNLP(const TrajPlannerConfig& config)
    : config_(config) {
  nlp_config_ = {{"ipopt", Dict({
#ifdef WITH_HSL
                               {"linear_solver", "ma27"},
                               {"linear_solver", "mumps"},
#else
#endif
                               {"print_level", 5},
                           })}};

  BuildNLP();
}

void TrajectoryNLP::BuildNLP() {
  double tf = config_.tf;
  //符号变量
  //每段轨迹有一个变量
  SX x = SX::sym("x", config_.nfe);
  SX y = SX::sym("y", config_.nfe);
  SX theta = SX::sym("theta", config_.nfe);
  SX v = SX::sym("v", config_.nfe);
  SX phi = SX::sym("phi", config_.nfe);
  SX a = SX::sym("a", config_.nfe);
  SX omega = SX::sym("omega", config_.nfe);
  SX jerk = SX::sym("jerk", config_.nfe);

  SX xf = SX::sym("xf", config_.nfe);
  SX yf = SX::sym("yf", config_.nfe);
  SX xr = SX::sym("xr", config_.nfe);
  SX yr = SX::sym("yr", config_.nfe);

  SX p_inf_w = SX::sym("inf_w");
  // SX p_ref_x = SX::sym("ref_x", config_.nfe, 1);
  // SX p_ref_y = SX::sym("ref_y", config_.nfe, 1);
  SX p_ref_theta = SX::sym("ref_theta", config_.nfe, 1);

  auto hi = tf / config_.nfe;
  //切片，取矩阵中从i到n的元素
  auto prev = Slice(0, config_.nfe - 1);
  auto next = Slice(1, config_.nfe);
  //就是计算dxdt-v(t)*cos(theta(t))等等
  auto g_x_kin = x(next) - (x(prev) + hi * v(prev) * cos(theta(prev)));
  auto g_y_kin = y(next) - (y(prev) + hi * v(prev) * sin(theta(prev)));
  auto g_theta_kin =
      theta(next) - (theta(prev) + hi * v(prev) * tan(phi(prev)) /
                                       config_.vehicle.wheel_base);
  auto g_v_kin = v(next) - (v(prev) + hi * a(prev));
  auto g_phi_kin = phi(next) - (phi(prev) + hi * omega(prev));
  auto g_a_kin = a(next) - (a(prev) + hi * jerk(prev));

  auto g_xf_kin = xf - (x + config_.vehicle.f2x * cos(theta));
  auto g_yf_kin = yf - (y + config_.vehicle.f2x * sin(theta));
  auto g_xr_kin = xr - (x + config_.vehicle.r2x * cos(theta));
  auto g_yr_kin = yr - (y + config_.vehicle.r2x * sin(theta));

  auto infeasibility = sumsqr(SX::vertcat(
      {g_x_kin, g_y_kin, g_theta_kin, g_v_kin, g_phi_kin, g_a_kin, g_xf_kin,
                                            g_yf_kin, g_xr_kin, g_yr_kin}));

  // w_u 控制项权重
  SX f_obj =
        // sumsqr(x - p_ref_x) + sumsqr(y - p_ref_y) + 
        config_.opti_w_r_theta *
        sumsqr(theta - p_ref_theta) +
      config_.opti_w_u * (sumsqr(jerk) + config_.opti_w_rw * sumsqr(omega)) +
      p_inf_w * infeasibility;

  SX p = SX::vertcat({p_inf_w,  p_ref_theta});
  SX opti_x = SX::vertcat({x, y, theta, v, phi, a, omega, jerk, xf, yf, xr, yr});
  SXDict nlp = {{"x", opti_x}, {"p", p}, {"f", f_obj}};
  solver_ = nlpsol("solver", "ipopt", nlp, nlp_config_);
  //就是一个函数，将opti_x 按照infeasibility的函数映射，看值是不是趋于0(可行性)
  infeasibility_evaluator_ = Function("inf", {opti_x}, {infeasibility}, {});
}

double TrajectoryNLP::solve(double inf_w, const Constraints& constraints,
                            const States& guess,
                            const std::vector<TrajectoryPoint>& reference,
                            States& result) {
  //单位向量?or单位矩阵?
  auto identity = DM::ones(config_.nfe, 1);
  DM lb_x, lb_y, lb_theta, lb_v, lb_phi, lb_a, lb_omega, lb_jerk;
  DM ub_x, ub_y, ub_theta, ub_v, ub_phi, ub_a, ub_omega, ub_jerk;
  DM lb_xf, lb_yf, lb_xr, lb_yr;
  DM ub_xf, ub_yf, ub_xr, ub_yr;
  //初始化向量
  lb_x = -inf * identity;
  ub_x = inf * identity;
  lb_y = -inf * identity;
  ub_y = inf * identity;
  lb_theta = -inf * identity;
  ub_theta = inf * identity;
  lb_v = -config_.vehicle.max_velocity * identity;
  ub_v = config_.vehicle.max_velocity * identity;
  lb_phi = -config_.vehicle.phi_max * identity;
  ub_phi = config_.vehicle.phi_max * identity;
  lb_a = -config_.vehicle.max_acceleration * identity;
  ub_a = config_.vehicle.max_acceleration * identity;
  lb_omega = -config_.vehicle.omega_max * identity;
  ub_omega = config_.vehicle.omega_max * identity;
  lb_jerk = -config_.vehicle.jerk_max * identity;
  ub_jerk = config_.vehicle.jerk_max * identity;

  // boundary value constraints
  int end = config_.nfe - 1;
  //起点约束 上下界约束
  lb_x(0) = ub_x(0) = constraints.start_x;
  lb_y(0) = ub_y(0) = constraints.start_y;
  lb_theta(0) = ub_theta(0) = constraints.start_theta;
  lb_v(0) = ub_v(0) = constraints.start_v;
  lb_phi(0) = ub_phi(0) = constraints.start_phi;
  lb_a(0) = ub_a(0) = constraints.start_a;
  lb_omega(0) = ub_omega(0) = constraints.start_omega;

  for (int i = 1; i < config_.nfe - 1; i++) {
    if (reference[i - 1].velocity < 0 && reference[i].velocity < 0) {
      ub_v(i) = 0.0;
    } else {
      lb_v(i) = 0.0;
    }

    // lb_x(i) = reference[i].x - 2;
    // ub_x(i) = reference[i].x + 2;
    // lb_y(i) = reference[i].y - 2;
    // ub_y(i) = reference[i].y + 2;
  }

  lb_xf = lb_yf = lb_xr = lb_yr = -inf * identity;
  ub_xf = ub_yf = ub_xr = ub_yr = inf * identity;
  for (int i = 1; i < config_.nfe; i++) {
    lb_xf(i) = constraints.front_bound[i][0];
    ub_xf(i) = constraints.front_bound[i][1];
    lb_yf(i) = constraints.front_bound[i][2];
    ub_yf(i) = constraints.front_bound[i][3];

    lb_xr(i) = constraints.rear_bound[i][0];
    ub_xr(i) = constraints.rear_bound[i][1];
    lb_yr(i) = constraints.rear_bound[i][2];
    ub_yr(i) = constraints.rear_bound[i][3];
  }

  //终点约束 只有加速度，Jerk, omega约束 不同问题要改变
  lb_x(end) = ub_x(end) = reference[config_.nfe - 1].x;
  lb_y(end) = ub_y(end) = reference[config_.nfe - 1].y;
  lb_theta(end) = ub_theta(end) = reference[config_.nfe - 1].theta;
  lb_v(end) = ub_v(end) = 0.0;
  lb_phi(end) = ub_phi(end) = 0.0;
  lb_a(end) = ub_a(end) = 0.0;
  lb_omega(end) = ub_omega(end) = 0.0;
  lb_jerk(end) = ub_jerk(end) = 0.0;

  DMDict arg, res;
  //变量边界
  arg["lbx"] = DM::vertcat(
      {lb_x, lb_y, lb_theta, lb_v, lb_phi, lb_a, lb_omega, lb_jerk, lb_xf, lb_yf, lb_xr, lb_yr});
  arg["ubx"] = DM::vertcat(
      {ub_x, ub_y, ub_theta, ub_v, ub_phi, ub_a, ub_omega, ub_jerk, ub_xf, ub_yf, ub_xr, ub_yr});
  //初值
  arg["x0"] = DM::vertcat({guess.x, guess.y, guess.theta, guess.v, guess.phi,
                           guess.a, guess.omega, guess.jerk, guess.xf, guess.yf, guess.xr,
     guess.yr});
  //可行性约束权重
    DM ref_x(config_.nfe, 1), ref_y(config_.nfe, 1), ref_theta(config_.nfe,1); 
    for (int i = 0; i < config_.nfe; i++) {
      ref_x(i) = reference.data()[i].x;
      ref_y(i) = reference.data()[i].y;
      ref_theta(i) = reference.data()[i].theta;
    }
  arg["p"] = DM::vertcat({inf_w, ref_theta});
  res = solver_(arg);
  DM opt = res.at("x");
  result.x.resize(config_.nfe);
  result.y.resize(config_.nfe);
  result.theta.resize(config_.nfe);
  result.v.resize(config_.nfe);
  result.phi.resize(config_.nfe);
  result.a.resize(config_.nfe);
  result.omega.resize(config_.nfe);
  result.jerk.resize(config_.nfe);
  result.xf.resize(config_.nfe);
  result.yf.resize(config_.nfe);
  result.xr.resize(config_.nfe);
  result.yr.resize(config_.nfe);
  for (int i = 0; i < config_.nfe; i++) {
    result.x[i] = double(opt(i, 0));
    result.y[i] = double(opt(config_.nfe + i, 0));
    result.theta[i] = double(opt(2 * config_.nfe + i, 0));
    result.v[i] = double(opt(3 * config_.nfe + i, 0));
    result.phi[i] = double(opt(4 * config_.nfe + i, 0));
    result.a[i] = double(opt(5 * config_.nfe + i, 0));
    result.omega[i] = double(opt(6 * config_.nfe + i, 0));
    result.jerk[i] = double(opt(7 * config_.nfe + i, 0));
    result.xf[i] = double(opt(8 * config_.nfe + i, 0));
    result.yf[i] = double(opt(9 * config_.nfe + i, 0));
    result.xr[i] = double(opt(10 * config_.nfe + i, 0));
    result.yr[i] = double(opt(11 * config_.nfe + i, 0));
  }

  //可行性分析
  std::vector<DM> arg_in = {opt};
  auto arg_out = infeasibility_evaluator_(arg_in);
  //返回infeasibility惩罚项的值
  return arg_out.front()->at(0);
}
