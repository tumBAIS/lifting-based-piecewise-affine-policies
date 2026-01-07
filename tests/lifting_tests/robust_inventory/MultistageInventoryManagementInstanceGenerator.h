#ifndef ROBUSTOPTIMIZATION_MULTISTAGEINVENTORYMANAGEMENTINSTANCEGENERATOR_H
#define ROBUSTOPTIMIZATION_MULTISTAGEINVENTORYMANAGEMENTINSTANCEGENERATOR_H

#include "../../test_helpers/InstanceGeneratorBase.h"

namespace testing {

class MultistageInventoryManagementInstanceGenerator : public InstanceGeneratorBase {

public:
    void add_num_stages(size_t num_stages);

    void add_alpha(double alpha);

    void add_overage_cost(double scale);

    void add_uncertainty_scale_function(std::function<double(size_t)> const& generator, std::string const& description);

    void add_end_of_horizon_scale_function(std::function<double(size_t)> const& generator, std::string const& description);


private:
    std::string descriptions_test_specific() const override;

    std::unique_ptr<robust_model::ROModel> generate_instance() override;

    std::string instance_description_test_specific() override;

    bool increment_test_specific() override;

private:
    double const _mu = 200;
    double const _order_cost = .1;
    double const _underage_cost = .2;

    double const _early_order_cost = 0.01;

    std::vector<double> _alphas;
    std::vector<size_t> _num_stages;
    std::vector<double> _overage_costs;
    std::vector<std::pair<std::function<double(size_t)>, std::string>> _uncertainty_scales;
    std::vector<std::pair<std::function<double(size_t)>, std::string>> _end_of_horizon_scales;
    std::vector<robust_model::RoAffineExpression::UncertaintyBehaviour> _objective_uncertainty_behaviours = {
            robust_model::RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE,
            robust_model::RoAffineExpression::UncertaintyBehaviour::STOCHASTIC
    };
    size_t _alphas_id = 0,
            _num_stages_id = 0,
            _overage_cost_id = 0,
            _uncertainty_scale_id = 0,
            _end_of_horizon_scale_id = 0,
            _objective_uncertainty_behaviours_id = 0;

};

}

#endif //ROBUSTOPTIMIZATION_MULTISTAGEINVENTORYMANAGEMENTINSTANCEGENERATOR_H
