#pragma once

#include "parameters.cpp"
#include "FootstepPlan.hpp"
#include <fstream>
#include "types.hpp"

double ramp(int time, int start);

CandidateMotion computeCandidateMotion(FootstepPlan* plan, State current, int time, int C, int P);
