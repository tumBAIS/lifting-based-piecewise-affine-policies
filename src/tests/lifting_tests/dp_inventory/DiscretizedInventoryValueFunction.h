
#ifndef PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_DISCRETIZEDINVENTORYVALUEFUNCTION_H
#define PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_DISCRETIZEDINVENTORYVALUEFUNCTION_H

#include <vector>

namespace testing {


class DiscretizedInventoryValueFunction {
public:
    DiscretizedInventoryValueFunction(
            size_t max_squared_past_demand,
            size_t max_periods,
            size_t max_backlog_demand,
            int min_inventory,
            int max_inventory
    );

    double value(
            size_t squared_past_demand,
            int past_demand,
            size_t total_backlogged_demand,
            int inventory
    ) const;

    double& value(
            size_t squared_past_demand,
            int past_demand,
            size_t total_backlogged_demand,
            int inventory);

    int min_past_demand(size_t const squared_past_demand) const;

    int max_past_demand(size_t const squared_past_demand) const;

public:
    size_t const max_squared_past_demand;
    size_t const max_periods;
    size_t const max_backlog_demand;
    int const min_inventory;
    int const max_inventory;
    std::vector<std::vector<std::vector<std::vector<double>>>> values;

};

class DiscretizedInventoryValueFunctionWithDemandTransition : public DiscretizedInventoryValueFunction {
public:
    DiscretizedInventoryValueFunctionWithDemandTransition(
            size_t max_squared_past_demand,
            size_t max_periods,
            size_t max_backlog_demand,
            int min_inventory,
            int max_inventory,
            double holding_cost,
            double backlogging_cost,
            size_t const_demand,
            double old_demand_factor);

    double value_after_demand(
            size_t squared_past_demand,
            int past_demand,
            size_t total_backlogged_demand,
            int inventory,
            int demand) const;

private:
    double const _holding_cost;
    double const _backlogging_cost;
    size_t const _const_demand;
    double const _old_demand_factor;
};


}

#endif //PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_DISCRETIZEDINVENTORYVALUEFUNCTION_H
