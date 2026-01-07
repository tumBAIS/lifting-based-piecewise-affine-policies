
#ifndef PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_DPINVENTORYSOLVER_H
#define PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_DPINVENTORYSOLVER_H

#include "DiscretizedInventoryValueFunction.h"

namespace testing {

class DPInventorySolver {
public:
    DPInventorySolver(
            size_t num_periods, double old_demand_factor, double discretization, bool robust, double const_replenishment
    );

    double compute_value();

private:
    std::vector<double> demand_probabilities(size_t dimension, double radius) const;

    void transition_demand_phase(DiscretizedInventoryValueFunction& pre_demand_values,
                                 DiscretizedInventoryValueFunctionWithDemandTransition const& post_demand_values,
                                 size_t const periods_to_go) const;

    void transition_replenish_phase(DiscretizedInventoryValueFunction const& pre_demand_values,
                                    DiscretizedInventoryValueFunctionWithDemandTransition& post_demand_values,
                                    size_t const periods_to_go) const;


private:
    bool const _robust;
    int const _num_periods;
    double const _old_demand_factor;
    double const _discretization;
    double const _sq_discretization;

    int const _const_demand;
    int const _const_replenishment;

    double const _holding_cost;
    double const _backlogging_cost;
    double const _final_backlog_cost;
    double const _adjustment_cost;
    double const _const_replenishment_cost;
    int const _max_sq_past_demand;
    int const _max_past_demand;
    int const _max_backlog;
};


}

#endif //PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_DPINVENTORYSOLVER_H
