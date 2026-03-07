#ifndef FEASIBILITYCHECKER_H
#define FEASIBILITYCHECKER_H
#pragma once

#include "structures.h"
#include "globals.h"
#include <vector>

enum class EvaluationMode
{
    STRICT,
    PENALTY
};

class FeasibilityChecker
{
private:
    const std::vector<Node> &nodes;
    const std::vector<Request> &requests;
    const std::vector<Vehicle> &vehicles;
    int max_global_capacity;

public:
    FeasibilityChecker(
        const std::vector<Node> &n,
        const std::vector<Request> &r,
        const std::vector<Vehicle> &v,
        int global_cap,
        EvaluationMode m = EvaluationMode::PENALTY);

    bool checkInsert(
        const std::vector<int> &current_route_ids,
        int vehicle_idx,
        int pickup_node_id,
        int delivery_node_id,
        int pickup_pos,
        int delivery_pos);

    bool runEightStepEvaluation(const std::vector<int> &route_ids, int veh_idx);

    long long getPenalty() const { return total_penalty; }

    EvaluationMode mode;
    long long total_penalty;

    // Penalty Multipliers (Tweak these to tell the solver which rules are most important)
    long long penalty_time_window = tot_speed_per_km * 1000; // Per minute late
    long long penalty_premium_vehicle = 5000000;             // Flat penalty for wrong vehicle type
    long long penalty_sharing_preference = 5000000;          // Flat penalty for violating max_shared_with
};

#endif // FEASIBILITYCHECKER_H