#include <random>
#include "MultistageInventoryManagementInstanceGeneratorServiceLevel.h"


namespace testing {

std::string MultistageInventoryManagementInstanceGeneratorServiceLevel::descriptions_test_specific() const {
    return "uncertainty;num_stages;alpha;service_level;";
}

std::unique_ptr<robust_model::ROModel> MultistageInventoryManagementInstanceGeneratorServiceLevel::generate_instance() {
    auto model_ptr = std::make_unique<robust_model::ROModel>("Model");
    auto& model = *model_ptr;

    size_t const T = _num_stages.at(_num_stages_id);
    double const alpha = _alphas.at(_alphas_id);
    double service_level = _service_levels.at(_service_level_id);
    double const nu = _mu / sqrt(T);

    auto const uncertainties = model.add_uncertainty_variables_for_each_period(T, "UncertainDemand", 1, -1, 1);

    auto const early_ordering_quantities = model.add_decision_variables(T, "EarlyOrder",
                                                                        0, 0, robust_model::NO_VARIABLE_UB);
    auto const reordering_quantities = model.add_decision_variables_for_each_period(T, "ReorderQuantity", 0, 0,
                                                                                    _max_order);
    auto const inventory = model.add_decision_variables_for_each_period(T + 1, "Inventory", 0,
                                                                        robust_model::NO_VARIABLE_LB,
                                                                        robust_model::NO_VARIABLE_UB);
    auto const overage_quantity = model.add_decision_variables_for_each_period(
            T, "OverageQuantity", 1, 0, robust_model::NO_VARIABLE_UB);
    auto const underage_quantity = model.add_decision_variables_for_each_period(
            T, "UnderageQuantity", 1, 0, robust_model::NO_VARIABLE_UB);

    robust_model::AffineExpression<robust_model::UncertaintyVariable::Reference> total_demand;
    for (size_t t = 0; t < T; ++t) {
        robust_model::AffineExpression<robust_model::UncertaintyVariable::Reference> demand;
        for (size_t to = 0; to < t; ++to) {
            demand += alpha * uncertainties.at(to) * nu;
        }
        demand += uncertainties.at(t) * nu + _mu;
        total_demand += demand;
        model.add_constraint(
                inventory.at(t + 1) ==
                inventory.at(t)
                + early_ordering_quantities.at(t)
                + reordering_quantities.at(t)
                - demand,
                "FlowConservation");
        model.add_constraint(
                overage_quantity.at(t) >= inventory.at(t + 1), "OveragePenalty");
        model.add_constraint(
                underage_quantity.at(t) >= -inventory.at(t + 1), "UnderagePenalty");
    }
    model.add_constraint(inventory.at(0) == 0, "NoStartingInventory");
    model.add_constraint(
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(underage_quantity) <=
            service_level * total_demand,
            "ServiceLevel"
    );
    auto obj = robust_model::RoAffineExpression(
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(overage_quantity) *
            _overage_cost +
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(
                    reordering_quantities) * _order_cost
            + underage_quantity.at(T - 1) * _order_cost
    );
    obj.set_multi_uncertainty_behaviour(_objective_uncertainty_behaviours.at(_objective_uncertainty_behaviours_id));
    model.set_objective(
            obj,
            robust_model::ObjectiveSense::MIN
    );
    return model_ptr;
}

std::string MultistageInventoryManagementInstanceGeneratorServiceLevel::instance_description_test_specific() {
    std::string s;
    s += robust_model::RoAffineExpression::to_string(
            _objective_uncertainty_behaviours.at(_objective_uncertainty_behaviours_id)) + ";";
    s += std::to_string(_num_stages.at(_num_stages_id)) + ";";
    s += std::to_string(_alphas.at(_alphas_id)) + ";";
    s += std::to_string(_service_levels.at(_service_level_id)) + ";";
    return s;
}

bool MultistageInventoryManagementInstanceGeneratorServiceLevel::increment_test_specific() {
    if (++_alphas_id < _alphas.size()) {
        return true;
    }
    _alphas_id = 0;
    if (++_service_level_id < _service_levels.size()) {
        return true;
    }
    _service_level_id = 0;
    if (++_num_stages_id < _num_stages.size()) {
        return true;
    }
    _num_stages_id = 0;
    if (++_objective_uncertainty_behaviours_id < _objective_uncertainty_behaviours.size()) {
        return true;
    }
    _objective_uncertainty_behaviours_id = 0;

    return false;
}

void MultistageInventoryManagementInstanceGeneratorServiceLevel::add_num_stages(size_t num_stages) {
    _num_stages.emplace_back(num_stages);
}

void MultistageInventoryManagementInstanceGeneratorServiceLevel::add_alpha(double alpha) {
    _alphas.emplace_back(alpha);
}

void MultistageInventoryManagementInstanceGeneratorServiceLevel::add_service_level(double scale) {
    _service_levels.emplace_back(scale);
}

}
