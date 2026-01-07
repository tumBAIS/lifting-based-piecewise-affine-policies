#include <cmath>
#include <numbers>

#include "DPInventorySolver.h"

namespace testing {

DPInventorySolver::DPInventorySolver(
        size_t num_periods, double old_demand_factor, double discretization, bool robust, double const_replenishment
) :
        _robust(robust),
        _num_periods(int(num_periods)),
        _old_demand_factor(old_demand_factor),
        _discretization(discretization),
        _sq_discretization(discretization * discretization),
        _const_demand(int(200 / discretization)),
        _const_replenishment(_const_demand + int((const_replenishment - 200.) / discretization)),
        _holding_cost(discretization * .04),
        _backlogging_cost(discretization * .2),
        _final_backlog_cost(discretization * 1.8),
        _adjustment_cost(discretization * .1),
        _const_replenishment_cost(discretization * .01),
        _max_sq_past_demand(_const_demand * _const_demand / _num_periods),
        _max_past_demand(_const_demand),
        _max_backlog(int(_const_demand * std::sqrt(_num_periods) * .2)) {}


std::vector<double> DPInventorySolver::demand_probabilities(size_t dimension, double radius) const {
    if (radius == 0.) {
        return {1.};
    }

    double a = std::tgamma(dimension / 2 + 1) / (std::sqrt(std::numbers::pi) * std::tgamma((dimension + 1) / 2));
    double probability_sum = 0;
    double demand = -int(radius);
    std::vector<double> probabilities;
    probabilities.reserve(int(radius) + 1);
    while (demand <= radius) {
        probabilities.push_back(
                (a / radius) * std::pow(1 - std::pow(demand / radius, 2), (dimension - 1) / 2)
        );
        probability_sum += probabilities.back();
        ++demand;
    }
    for (auto& probability: probabilities) {
        probability /= probability_sum;
    }
    return probabilities;
}

void DPInventorySolver::transition_demand_phase(DiscretizedInventoryValueFunction& pre_demand_values,
                                                DiscretizedInventoryValueFunctionWithDemandTransition const& post_demand_values,
                                                size_t const periods_to_go) const {
    for (size_t squared_past_demand = 0;
         squared_past_demand <= pre_demand_values.max_squared_past_demand;
         ++squared_past_demand) {
        for (int past_demand = pre_demand_values.min_past_demand(squared_past_demand);
             past_demand <= pre_demand_values.max_past_demand(squared_past_demand);
             ++past_demand) {
            if (((_num_periods == periods_to_go) and ((past_demand != 0) or (squared_past_demand > 0))) or
                ((_num_periods > periods_to_go) and
                 (squared_past_demand < double(past_demand * past_demand) / (_num_periods - periods_to_go)))
                    ) {
                continue;
            }
            auto const probabilities = demand_probabilities(
                    periods_to_go,
                    std::sqrt(_max_sq_past_demand - squared_past_demand));
            for (size_t backlog = 0; backlog <= pre_demand_values.max_backlog_demand; ++backlog) {
                for (int inventory = pre_demand_values.min_inventory;
                     inventory <= pre_demand_values.max_inventory;
                     ++inventory) {
                    auto& val = pre_demand_values.value(squared_past_demand, past_demand, backlog, inventory);
                    val = 0;
                    int max_abs_demand = int(probabilities.size() - 1) / 2;
                    for (int demand = -max_abs_demand; demand <= max_abs_demand; ++demand) {
                        auto const next_cost = post_demand_values.value_after_demand(
                                squared_past_demand, past_demand, backlog, inventory,
                                demand);
                        if (_robust) {
                            val = std::max(val, next_cost);
                        } else {
                            val += probabilities[demand + max_abs_demand] * next_cost;
                        }
                        if (std::isinf(val))
                            break;
                    }
                }
            }
        }
    }
}

void DPInventorySolver::transition_replenish_phase(DiscretizedInventoryValueFunction const& pre_demand_values,
                                                   DiscretizedInventoryValueFunctionWithDemandTransition& post_demand_values,
                                                   size_t const periods_to_go) const {
    for (size_t squared_past_demand = 0;
         squared_past_demand <= post_demand_values.max_squared_past_demand;
         ++squared_past_demand) {
        for (int past_demand = pre_demand_values.min_past_demand(squared_past_demand);
             past_demand <= pre_demand_values.max_past_demand(squared_past_demand);
             ++past_demand) {
            if (((_num_periods == periods_to_go) and ((past_demand != 0) or (squared_past_demand > 0))) or
                ((_num_periods > periods_to_go) and
                 (squared_past_demand < double(past_demand * past_demand) / (_num_periods - periods_to_go)))
                    ) {
                continue;
            }
            for (size_t backlog = 0; backlog <= post_demand_values.max_backlog_demand; ++backlog) {
                for (int inventory = post_demand_values.min_inventory;
                     inventory <= post_demand_values.max_inventory;
                     ++inventory) {
                    auto& val = post_demand_values.value(squared_past_demand, past_demand, backlog, inventory);
                    val = std::numeric_limits<double>::infinity();
                    for (int next_inventory = pre_demand_values.min_inventory;
                         next_inventory <= pre_demand_values.max_inventory;
                         ++next_inventory) {
                        int const adjusted_replenishment = next_inventory - inventory - _const_replenishment;
                        val = std::min(
                                val,
                                pre_demand_values.value(squared_past_demand, past_demand, backlog, next_inventory) +
                                std::abs(adjusted_replenishment) * _adjustment_cost);
                    }
                }
            }
        }
    }
}

double DPInventorySolver::compute_value() {

    DiscretizedInventoryValueFunction pre_demand_values(
            _max_sq_past_demand,
            _num_periods,
            _max_backlog,
            0,
            2 * _const_demand
    );

    DiscretizedInventoryValueFunctionWithDemandTransition post_demand_values(
            _max_sq_past_demand,
            _num_periods,
            _max_backlog,
            -_const_demand,
            _const_demand,
            _holding_cost,
            _backlogging_cost,
            _const_demand,
            _old_demand_factor
    );


    for (size_t squared_past_demand = 0;
         squared_past_demand <= post_demand_values.max_squared_past_demand;
         ++squared_past_demand) {
        for (int past_demand = pre_demand_values.min_past_demand(squared_past_demand);
             past_demand <= pre_demand_values.max_past_demand(squared_past_demand);
             ++past_demand) {
            for (size_t backlog = 0; backlog <= post_demand_values.max_backlog_demand; ++backlog) {
                for (int inventory = post_demand_values.min_inventory;
                     inventory <= post_demand_values.max_inventory;
                     ++inventory) {
                    post_demand_values.value(squared_past_demand, past_demand, backlog, inventory) =
                            std::max(-inventory, 0) * _final_backlog_cost;
                }
            }
        }
    }

    for (int t = 1; t <= _num_periods; ++t) {
        transition_demand_phase(pre_demand_values, post_demand_values, t);
        transition_replenish_phase(pre_demand_values, post_demand_values, t);
    }

    return post_demand_values.value(0, 0, 0, 0) + _num_periods * _const_replenishment_cost * _const_replenishment;
}


}
