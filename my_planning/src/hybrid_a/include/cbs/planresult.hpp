#pragma once

#include <vector>

template <typename State, typename Action, typename Cost>
struct PlanResult {
  std::vector<State> states;
  std::vector<Action> actions;
  Cost cost;
  Cost fmin;
  int start_time;
};
