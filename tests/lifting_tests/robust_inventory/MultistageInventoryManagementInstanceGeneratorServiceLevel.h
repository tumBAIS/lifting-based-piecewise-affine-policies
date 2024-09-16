#ifndef PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_MULTISTAGEINVENTORYMANAGEMENTINSTANCEGENERATORSERVICELEVEL_H
#define PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_MULTISTAGEINVENTORYMANAGEMENTINSTANCEGENERATORSERVICELEVEL_H

#include "../../test_helpers/InstanceGeneratorBase.h"

namespace testing {

class MultistageInventoryManagementInstanceGeneratorServiceLevel : public InstanceGeneratorBase {


public:
    void add_num_stages(size_t num_stages);

    void add_alpha(double alpha);

    void add_service_level(double scale);

private:
    std::string descriptions_test_specific() const override;

    std::unique_ptr<robust_model::ROModel> generate_instance() override;

    std::string instance_description_test_specific() override;

    bool increment_test_specific() override;

private:
    double const _mu = 200;
    double const _max_order = 260;
    double const _order_cost = .1;
    double const _overage_cost = .02;

    std::vector<double> _alphas;
    std::vector<size_t> _num_stages;
    std::vector<double> _service_levels;
    std::vector<robust_model::RoAffineExpression::UncertaintyBehaviour> _objective_uncertainty_behaviours = {
            robust_model::RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE,
            robust_model::RoAffineExpression::UncertaintyBehaviour::STOCHASTIC
    };
    size_t _alphas_id = 0,
            _num_stages_id = 0,
            _service_level_id = 0,
            _objective_uncertainty_behaviours_id = 0;
};

}

#endif //PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_MULTISTAGEINVENTORYMANAGEMENTINSTANCEGENERATORSERVICELEVEL_H
