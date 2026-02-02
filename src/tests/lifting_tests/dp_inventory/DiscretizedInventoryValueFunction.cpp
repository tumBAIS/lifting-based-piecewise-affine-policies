#include <cmath>

#include "DiscretizedInventoryValueFunction.h"

namespace testing {
DiscretizedInventoryValueFunction::DiscretizedInventoryValueFunction(
        size_t max_squared_past_demand,
        size_t max_periods,
        size_t max_backlog_demand,
        int min_inventory,
        int max_inventory
) :
        max_squared_past_demand(max_squared_past_demand),
        max_periods(max_periods),
        max_backlog_demand(max_backlog_demand),
        min_inventory(min_inventory),
        max_inventory(max_inventory) {
    std::vector<std::vector<double>> backlog_inventory_vec(
            max_backlog_demand + 1, std::vector<double>(max_inventory - min_inventory + 1, 0.));
    for (size_t squared_past_demand = 0; squared_past_demand <= max_squared_past_demand; ++squared_past_demand) {
        values.emplace_back(
                max_past_demand(squared_past_demand) - min_past_demand(squared_past_demand) + 1,
                backlog_inventory_vec
        );
    }
}

double DiscretizedInventoryValueFunction::value(
        size_t squared_past_demand,
        int past_demand,
        size_t total_backlogged_demand,
        int inventory
) const {
    if ((inventory < min_inventory) or (inventory > max_inventory) or
        (total_backlogged_demand > max_backlog_demand)) {
        return std::numeric_limits<double>::infinity();
    }
    return values[
            squared_past_demand][
            past_demand - min_past_demand(squared_past_demand)][
            total_backlogged_demand][
            inventory - min_inventory];
}

double& DiscretizedInventoryValueFunction::value(
        size_t squared_past_demand,
        int past_demand,
        size_t total_backlogged_demand,
        int inventory) {
    return values[
            squared_past_demand][
            past_demand - min_past_demand(squared_past_demand)][
            total_backlogged_demand][
            inventory - min_inventory];
}

int DiscretizedInventoryValueFunction::min_past_demand(size_t const squared_past_demand) const {
    return -max_past_demand(squared_past_demand);
}

int DiscretizedInventoryValueFunction::max_past_demand(size_t const squared_past_demand) const {
    return int(std::sqrt(squared_past_demand * max_periods));
}


DiscretizedInventoryValueFunctionWithDemandTransition::DiscretizedInventoryValueFunctionWithDemandTransition(
        size_t max_squared_past_demand,
        size_t max_periods,
        size_t max_backlog_demand,
        int min_inventory,
        int max_inventory,
        double holding_cost,
        double backlogging_cost,
        size_t const_demand,
        double old_demand_factor) :
        DiscretizedInventoryValueFunction(max_squared_past_demand,
                                          max_periods,
                                          max_backlog_demand,
                                          min_inventory,
                                          max_inventory
        ),
        _holding_cost(holding_cost), _backlogging_cost(backlogging_cost),
        _const_demand(const_demand), _old_demand_factor(old_demand_factor) {}

double DiscretizedInventoryValueFunctionWithDemandTransition::value_after_demand(
        size_t squared_past_demand,
        int past_demand,
        size_t total_backlogged_demand,
        int inventory,
        int demand) const {
    double const total_demand = double(past_demand) * _old_demand_factor + double(demand + _const_demand);
    double const new_inventory = inventory - total_demand;
    return value(
            squared_past_demand + demand * demand,
            past_demand + demand,
            total_backlogged_demand + size_t(std::max(-new_inventory, 0.)),
            int(new_inventory)
    ) + std::max(new_inventory, 0.) * _holding_cost + std::max(-new_inventory, 0.) * _backlogging_cost;
}


}

