#include "MultistageInventoryManagementInstanceGenerator.h"
#include "../../../solvers/uncertainty_solver/UncertaintySolver.h"
#include "../../../models/basic_model_objects/SolutionRealization.h"
#include <random>

namespace testing {

std::string MultistageInventoryManagementInstanceGenerator::descriptions_test_specific() const {
    return "uncertainty;num_stages;alpha;overage_cost;uncertainty_scale;end_of_horizon_scale;";
}

std::unique_ptr<robust_model::ROModel> MultistageInventoryManagementInstanceGenerator::generate_instance() {
    auto model_ptr = std::make_unique<robust_model::ROModel>("Model");
    auto& model = *model_ptr;

    std::random_device rd;
    std::mt19937 gen(rd());

    size_t const T = _num_stages.at(_num_stages_id);
    double const alpha = _alphas.at(_alphas_id);
    double const nu = _mu / _uncertainty_scales.at(_uncertainty_scale_id).first(T);
    double const overage_cost = _overage_costs.at(_overage_cost_id);
    double const underage_cost_last = _underage_cost * _end_of_horizon_scales.at(_end_of_horizon_scale_id).first(T);

    auto const uncertainties = model.add_uncertainty_variables_for_each_period(
            T, "UncertainDemand", 1, -_mu, _mu);
    std::vector<robust_model::AffineExpression<robust_model::UncertaintyVariable::Reference>> transformed_uncertainty;
    for (size_t t = 0; t < T; ++t) {
        robust_model::AffineExpression<robust_model::UncertaintyVariable::Reference> expr(uncertainties.at(t));
        double scale = -alpha;
        for (size_t tr = 0; tr < t; ++tr) {
            expr += scale * uncertainties.at(t - tr - 1);
            scale -= alpha * scale;
        }
        transformed_uncertainty.emplace_back(expr);
    }
    model.add_uncertainty_constraint(
            {robust_model::SOCExpression<robust_model::UncertaintyVariable>(
                    robust_model::NormedAffineVector<robust_model::UncertaintyVariable>(
                            robust_model::VectorNormType::Two,
                            transformed_uncertainty), 0) <= nu,
             "TransformedBall"}
    );


    model.non_const_uncertainty_set().set_uncertainty_sampler(
            [&]() {
                auto base_uncertainty = std::make_unique<robust_model::UncertaintySet>(model);
                std::vector<robust_model::SOCExpression<robust_model::UncertaintyVariable>> transformation;
                robust_model::SOCExpression<robust_model::UncertaintyVariable> previous_uncertainty;
                for (std::size_t t = 0; t < T; ++t) {
                    auto const lastvar = base_uncertainty->add_variable("base_uncertainty" + std::to_string(t), t, -nu,
                                                                        nu);
                    transformation.emplace_back(previous_uncertainty + lastvar);
                    previous_uncertainty += alpha * lastvar;
                }
                base_uncertainty->add_special_type_constraint(robust_model::UncertaintySet::SpecialSetType::BALL, nu);
                auto base_uncertainty_sampler = std::make_unique<robust_model::UniformL2BallRejectUncertaintySampler>(
                        *base_uncertainty, nu, false);

                return std::make_unique<robust_model::OwningTransformedUncertaintySampler>(
                        model.uncertainty_set(),
                        std::move(base_uncertainty),
                        std::move(base_uncertainty_sampler),
                        std::move(transformation));
            }()
    );

    model.non_const_uncertainty_set().set_max_one_norm_k_active_lookup(
            [&]() {
                std::vector<double> lookup;
                robust_model::UncertaintySolver uncertainty_solver(model.uncertainty_set());
                uncertainty_solver.build();
                robust_model::AffineExpression<robust_model::UncertaintyVariable::Reference> last_sum;
                for (size_t t = 0; t < T; ++t) {
                    last_sum += uncertainties.at(T - t - 1);
                    uncertainty_solver.set_objective({robust_model::ObjectiveSense::MAX, last_sum});
                    uncertainty_solver.solve();
                    lookup.emplace_back(uncertainty_solver.objective_value());
                }
                return lookup;
            }()
    );

    model.non_const_uncertainty_set().set_pretend_symmetric_rotational_invariant(true);

    auto const early_ordering_quantity = model.add_decision_variable("EarlyOrdering", 0, 0);
    auto const early_ordering = std::vector<robust_model::DecisionVariable::Reference>(T, early_ordering_quantity);

    auto const overorder_quantities = model.add_decision_variables_for_each_period(
            T, "OverorderQuantity", 0, 0, 200);
    auto const underorder_quantities = model.add_decision_variables_for_each_period(
            T, "UnderorderQuantity", 0, 0, 200);
    auto const inventory = model.add_decision_variables_for_each_period(
            T + 1, "Inventory", 0);
    auto const overage = model.add_decision_variables_for_each_period(T, "Overage", 1, 0);
    auto const underage = model.add_decision_variables_for_each_period(T, "Underage", 1, 0);

    for (size_t t = 0; t < T; ++t) {
        auto const inventory_idx = inventory.at(t + 1);
        model.non_const_decision_variables().at(
                overage.at(t).raw_id()).set_exact_evaluation(
                [inventory_idx](robust_model::SolutionRealization const& sr) {
                    return std::max(sr.value(inventory_idx), 0.);
                });
        model.non_const_decision_variables().at(
                underage.at(t).raw_id()).set_exact_evaluation(
                [inventory_idx](robust_model::SolutionRealization const& sr) {
                    return std::max(-sr.value(inventory_idx), 0.);
                });
    }



    for (size_t t = 0; t < T; ++t) {
        robust_model::AffineExpression<robust_model::UncertaintyVariable::Reference> demand;
        demand += uncertainties.at(t) + _mu;
        model.add_constraint(
                inventory.at(t + 1) ==
                        inventory.at(t)
                        + overorder_quantities.at(t)
                        - underorder_quantities.at(t)
                        + early_ordering.at(t)
                - demand,
                "FlowConservation");
        model.add_constraint(
                overage.at(t) >= inventory.at(t + 1), "OveragePenalty");
        model.add_constraint(
                underage.at(t) >= -inventory.at(t + 1),
                "UnderagePenalty");
    }

    model.add_constraint(inventory.at(0) == 0, "NoStartingInventory");

    model.add_constraint(
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(underage) <=
            .2 * nu * double(T),
            "ServiceLevel"
    );

    auto obj = robust_model::RoAffineExpression(
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(overage) * overage_cost +
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(underage) * _underage_cost +
            underage.back() * (underage_cost_last - _underage_cost) +
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(
                    overorder_quantities) * _order_cost +
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(
                    underorder_quantities) * _order_cost +
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(
                    early_ordering) * _early_order_cost
    );
    obj.set_multi_uncertainty_behaviour(_objective_uncertainty_behaviours.at(_objective_uncertainty_behaviours_id));
    model.set_objective(
            obj,
            robust_model::ObjectiveSense::MIN
    );
    return model_ptr;
}

std::string MultistageInventoryManagementInstanceGenerator::instance_description_test_specific() {
    std::string s;
    s += robust_model::RoAffineExpression::to_string(
            _objective_uncertainty_behaviours.at(_objective_uncertainty_behaviours_id)) + ";";
    s += std::to_string(_num_stages.at(_num_stages_id)) + ";";
    s += std::to_string(_alphas.at(_alphas_id)) + ";";
    s += std::to_string(_overage_costs.at(_overage_cost_id)) + ";";
    s += _uncertainty_scales.at(_uncertainty_scale_id).second + ";";
    s += _end_of_horizon_scales.at(_end_of_horizon_scale_id).second + ";";
    return s;
}

bool MultistageInventoryManagementInstanceGenerator::increment_test_specific() {
    if (++_alphas_id < _alphas.size()) {
        return true;
    }
    _alphas_id = 0;
    if (++_overage_cost_id < _overage_costs.size()) {
        return true;
    }
    _overage_cost_id = 0;
    if (++_num_stages_id < _num_stages.size()) {
        return true;
    }
    _num_stages_id = 0;
    if (++_uncertainty_scale_id < _uncertainty_scales.size()) {
        return true;
    }
    _uncertainty_scale_id = 0;
    if (++_end_of_horizon_scale_id < _end_of_horizon_scales.size()) {
        return true;
    }
    _end_of_horizon_scale_id = 0;
    if (++_objective_uncertainty_behaviours_id < _objective_uncertainty_behaviours.size()) {
        return true;
    }
    _objective_uncertainty_behaviours_id = 0;

    return false;
}

void MultistageInventoryManagementInstanceGenerator::add_num_stages(size_t num_stages) {
    _num_stages.emplace_back(num_stages);
}

void MultistageInventoryManagementInstanceGenerator::add_alpha(double alpha) {
    _alphas.emplace_back(alpha);
}

void MultistageInventoryManagementInstanceGenerator::add_overage_cost(double scale) {
    _overage_costs.emplace_back(scale);
}

void MultistageInventoryManagementInstanceGenerator::add_uncertainty_scale_function(
        const std::function<double(size_t)>& generator,
        const std::string& description) {
    _uncertainty_scales.emplace_back(generator, description);
}

void MultistageInventoryManagementInstanceGenerator::add_end_of_horizon_scale_function(
        const std::function<double(size_t)>& generator, const std::string& description) {
    _end_of_horizon_scales.emplace_back(generator, description);
}

}