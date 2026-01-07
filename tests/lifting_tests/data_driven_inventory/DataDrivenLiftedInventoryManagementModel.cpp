#include "DataDrivenLiftedInventoryManagementModel.h"
#include <algorithm>

namespace data_models {


DataDrivenLiftedInventoryManagementModel::DataDrivenLiftedInventoryManagementModel(
        DataDrivenLiftedInventoryManagementModel::Parameter parameter, double radius) :
        DataDrivenLiftedInventoryManagementModel(parameter.mode, radius, parameter.overage_cost,
                                                 parameter.eoh_underage_cost,
                                                 parameter.num_lifting_parts) {}

DataDrivenLiftedInventoryManagementModel::DataDrivenLiftedInventoryManagementModel(
        DataDrivenLiftedInventoryManagementModel::Mode const mode, double const radius, double overage_cost,
        double eoh_underage_cost,
        size_t num_lifting_parts) :
        _mode(mode), _radius(radius), _num_lifting_parts(num_lifting_parts), _overage_cost(overage_cost),
        _underage_cost_last(eoh_underage_cost),
        _model("InventoryManagementModel") {
    helpers::exception_check(_mode == Mode::AFFINE or _num_lifting_parts > 1,
                             "Need to non trivial lifting, when lifting is used");
}

void DataDrivenLiftedInventoryManagementModel::build_ro_model(DataModelBase::SampleData const& training_data) {
    size_t const sample_size = training_data.size();
    size_t const T = training_data.front().size();

    auto const [lbs, ubs] = get_uncertainty_bounds(training_data);
    std::vector<robust_model::ROModel::UncertaintyReference> uncertainties;
    for (size_t t = 0; t < T; ++t) {
        uncertainties.emplace_back(_model.add_uncertainty_variable(
                "UncertaintDemand" + std::to_string(t), t + 1, lbs.at(t)-_radius, ubs.at(t)+_radius));
    }
    _model.non_const_uncertainty_set().replace_with_constraint_sample_sets(training_data, _radius);

    auto const early_ordering_quantity = _model.add_decision_variable("EarlyOrdering", 0, 0, 400);
    _early_ordering_quantities = std::vector<robust_model::DecisionVariable::Reference>(T, early_ordering_quantity);

    _order_deviation = _model.add_decision_variables_for_each_period(
            T, "ReorderQuantity", 0, -_max_order, _max_order);
    auto const abs_order_deviation = _model.add_decision_variables_for_each_period(
            T, "OrderDeviation", 0, 0, robust_model::NO_VARIABLE_UB);
    auto const inventory = _model.add_decision_variables_for_each_period(
            T + 1, "Inventory", 0);
    auto const overage = _model.add_decision_variables_for_each_period(T, "Penalty", 1, 0);
    auto const underage = _model.add_decision_variables_for_each_period(T, "Underage", 1, 0);

    for (size_t t = 0; t < T; ++t) {
        auto const inventory_idx = inventory.at(t + 1);
        _model.non_const_decision_variables().at(
                overage.at(t).raw_id()).set_exact_evaluation(
                [inventory_idx](robust_model::SolutionRealization const& sr) {
                    return std::max(sr.value(inventory_idx), 0.);
                });
        _model.non_const_decision_variables().at(
                underage.at(t).raw_id()).set_exact_evaluation(
                [inventory_idx](robust_model::SolutionRealization const& sr) {
                    return std::max(-sr.value(inventory_idx), 0.);
                });
    }

    for (size_t t = 0; t < T; ++t) {
        _model.add_constraint(
                inventory.at(t + 1) ==
                inventory.at(t)
                + _order_deviation.at(t)
                + _early_ordering_quantities.at(t)
                - uncertainties.at(t),
                "FlowConservation");
        _model.add_constraint(
                overage.at(t) >= inventory.at(t + 1), "OveragePenalty");
        _model.add_constraint(
                underage.at(t) >= -inventory.at(t + 1),
                "UnderagePenalty");
        _model.add_constraint(
                abs_order_deviation.at(t) >= _order_deviation.at(t),
                "AbsOrderDeviation");
        _model.add_constraint(
                abs_order_deviation.at(t) >= -_order_deviation.at(t),
                "AbsOrderDeviation");
    }

    _model.add_constraint(inventory.at(0) == 0, "NoStartingInventory");

    _model.add_constraint(
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(underage) <=
            max_total_backlog(_model.num_uvars()),
            "ServiceLevel"
    );

    auto obj = robust_model::RoAffineExpression(
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(overage) * _overage_cost +
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(underage) * _underage_cost +
            underage.back() * (_underage_cost_last - _underage_cost) +
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(
                    abs_order_deviation) * _deviation_cost +
            robust_model::LinearExpression<robust_model::DecisionVariable::Reference>::sum(
                    _early_ordering_quantities) * _early_order_cost
    );

    obj.set_multi_uncertainty_behaviour(robust_model::RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE);
    _model.set_objective(
            obj,
            robust_model::ObjectiveSense::MIN
    );
}

void DataDrivenLiftedInventoryManagementModel::solve_ro_model(SampleData const& training_data) {
    switch (_mode) {
        case Mode::AFFINE:
            _affine_model = std::make_unique<robust_model::AffineAdjustablePolicySolver>(_model);
            _affine_model->build();
            _affine_model->solve();
            break;
        case Mode::LIFTED_EQUIDISTANT:
            _lifting_model = std::make_unique<robust_model::LiftingPolicySolver>(_model);
            _lifting_model->add_equidistant_breakpoints(_num_lifting_parts);
            _lifting_model->build();
            _lifting_model->solve();
            break;
        case Mode::LIFTED_PERCENTILES:
            _lifting_model = std::make_unique<robust_model::LiftingPolicySolver>(_model);
            auto const break_points = calculate_equidistant_percentiles(_num_lifting_parts, training_data);
            for (auto const& uvar: _model.uncertainty_variables()) {
                _lifting_model->add_break_points(
                        break_points.at(uvar.id().raw_id()),
                        robust_model::SingleDirectionBreakPoints::BreakPointDirection{uvar.reference()});
            }
            _lifting_model->build();
            _lifting_model->solve();
            break;
    }
}

bool DataDrivenLiftedInventoryManagementModel::train(DataModelBase::SampleData const& training_data) {
    build_ro_model(training_data);
    solve_ro_model(training_data);
    return active_solver().has_solution();
}

ScoreOutput DataDrivenLiftedInventoryManagementModel::test(DataModelBase::SampleData const& test_data) const {
    std::vector<double> objs;
    std::vector<double> valids;
    for (auto const& sample: test_data) {
        auto const& [obj, val] = realization_objective(realization(sample));
        objs.emplace_back(obj);
        valids.emplace_back(val);
    }
    return {helpers::mean(objs), helpers::standard_deviation(objs), helpers::mean(valids)};
}

double DataDrivenLiftedInventoryManagementModel::train_time() const {
    switch (_mode) {
        case Mode::AFFINE:
            return _affine_model->runtime();
        case Mode::LIFTED_EQUIDISTANT:
            [[fallthrough]];
        case Mode::LIFTED_PERCENTILES:
            return _lifting_model->runtime();
    }
}

robust_model::SolutionRealization
DataDrivenLiftedInventoryManagementModel::realization(DataModelBase::DataPoint const& sample) const {
    switch (_mode) {
        case Mode::AFFINE:
            return _affine_model->specific_solution(sample);
        case Mode::LIFTED_EQUIDISTANT:
            [[fallthrough]];
        case Mode::LIFTED_PERCENTILES:
            return _lifting_model->specific_solution(sample);
    }
}

std::tuple<double, bool> DataDrivenLiftedInventoryManagementModel::realization_objective(
        robust_model::SolutionRealization const& realization) const {
    double inventory = 0;
    double total_backlog = 0;
    double obj = 0;
    size_t T = _order_deviation.size();
    for (size_t t = 0; t < T; ++t) {
        double early_order = realization.value(_early_ordering_quantities.at(t));
        double order_deviation = std::max(std::min(realization.value(_order_deviation.at(t)), _max_order), -_max_order);
        double demand = realization.value(_model.uncertainty_variables().at(t).id());
        inventory += order_deviation + early_order - demand;
        obj += _overage_cost * std::max(inventory, 0.);
        obj += (t + 1 == T ? _underage_cost_last : _underage_cost) * std::max(-inventory, 0.);
        obj += _early_order_cost * early_order + _deviation_cost * std::abs(order_deviation);
        total_backlog += std::max(-inventory, 0.);
    }

    return {obj, total_backlog <= max_total_backlog(_model.num_uvars())};
}

double DataDrivenLiftedInventoryManagementModel::max_total_backlog(size_t T) const {
    return .2 * 200 * std::sqrt(double(T));
}

std::vector<robust_model::SingleDirectionBreakPoints::BreakPointsSeries>
DataDrivenLiftedInventoryManagementModel::calculate_equidistant_percentiles(size_t num_pieces,
                                                                            DataModelBase::SampleData const& training_data) {
    std::vector<double> percentiles(num_pieces - 1);
    for (size_t i = 1; i < num_pieces; ++i) {
        percentiles[i - 1] = double(i) / double(num_pieces);
    }
    return calculate_percentiles(percentiles, training_data);
}

std::vector<robust_model::SingleDirectionBreakPoints::BreakPointsSeries>
DataDrivenLiftedInventoryManagementModel::calculate_percentiles(std::vector<double> percentiles,
                                                                DataModelBase::SampleData const& training_data) {
    std::vector<std::vector<double>> uvar_data(training_data.front().size(), std::vector<double>(training_data.size()));
    for (size_t i = 0; i < training_data.size(); ++i) {
        for (size_t j = 0; j < training_data[i].size(); ++j) {
            uvar_data[j][i] = training_data[i][j];
        }
    }
    std::vector<robust_model::SingleDirectionBreakPoints::BreakPointsSeries>
            percentile_values(uvar_data.size(),
                              robust_model::SingleDirectionBreakPoints::BreakPointsSeries(percentiles.size()));
    for (size_t i = 0; i < uvar_data.size(); ++i) {
        auto& data = uvar_data[i];
        std::sort(data.begin(), data.end());
        for (size_t j = 0; j < percentiles.size(); ++j) {
            double const p = percentiles[j];
            double index = p * static_cast<double>(data.size() - 1);
            size_t index_low = static_cast<size_t>(index);
            size_t index_high = index_low + 1 < data.size() ? index_low + 1 : index_low;

            double frac = index - double(index_low);
            percentile_values[i][j] = data[index_low] + (data[index_high] - data[index_low]) * frac;
        }
    }

    return percentile_values;
}

solvers::SolverBase const& DataDrivenLiftedInventoryManagementModel::active_solver() const {
    switch (_mode) {
        case Mode::AFFINE:
            return *_affine_model;
        case Mode::LIFTED_EQUIDISTANT:
            [[fallthrough]];
        case Mode::LIFTED_PERCENTILES:
            return *_lifting_model;
    }
}


}

