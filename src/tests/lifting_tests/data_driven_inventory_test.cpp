#include "../../models/ROModel.h"
#include "../../solvers/aro_policy_solvers/LiftingPolicySolver.h"
#include <iostream>
#include <random>
#include "../test_helpers/ParallelInstanceEvaluator.h"

#include "data_driven_inventory/DataDrivenLiftedInventoryManagementModel.h"
#include "../test_helpers/CVModel.h"
#include "../test_helpers/ParallelDataInstanceEvaluator.h"

struct Parameter {
    size_t const iteration;
    size_t const size;
    size_t const T;
    double const alpha;
    double const overage_cost;
    double const eoh_underage;
    bool const cross_validation;
};

class ParameterIterator : public data_models::ParameterIterator<Parameter> {
public:
    bool increment() override {
        if (not valid())
            return false;
        if (++_alpha_options_it < _alpha_options.size())
            return true;
        _alpha_options_it = 0;
        if (++_overage_options_it < _overage_options.size())
            return true;
        _overage_options_it = 0;
        if (++_eoh_underage_options_it < _eoh_underage_options.size())
            return true;
        _eoh_underage_options_it = 0;
        if (++_train_size_options_it < _train_size_options.size())
            return true;
        _train_size_options_it = 0;
        if (++_stage_num_options_it < _stage_num_options.size())
            return true;
        _stage_num_options_it = 0;
        helpers::global_logger << "Finished Iteration " + std::to_string(_iteration);
        if (++_iteration < _num_iterations)
            return true;
        _iteration = 0;
        if (not _cross_validate) {
            _cross_validate = true;
            return true;
        }
        invalidate();
        return false;
    }

    Parameter get_parameter() const override {
        return {_iteration, _train_size_options.at(_train_size_options_it),
                _stage_num_options.at(_stage_num_options_it),
                _alpha_options.at(_alpha_options_it),
                _overage_options.at(_overage_options_it),
                _eoh_underage_options.at(_eoh_underage_options_it),
                _cross_validate
        };
    }

private:
    size_t const _num_iterations = 200;
    size_t _iteration = 0;
    std::vector<size_t> const _stage_num_options = {5};
    size_t _stage_num_options_it = 0;
    std::vector<size_t> const _train_size_options = {10, 25, 50, 100};
    size_t _train_size_options_it = 0;
    std::vector<double> const _alpha_options = {0., .25, .5};
    size_t _alpha_options_it = 0;
    std::vector<double> const _overage_options = {0.04};
    size_t _overage_options_it = 0;
    std::vector<double> const _eoh_underage_options = {2.};
    size_t _eoh_underage_options_it = 0;
    bool _cross_validate = false;
};

std::vector<std::vector<double>> generate_data(size_t const size, size_t const T, double const alpha) {
    double const base_demand = 200;
    double const nu = base_demand/ std::sqrt(double(T));

    robust_model::ROModel dummy_model("DummyModel");
    dummy_model.add_uncertainty_variables(T, "demand", 0, 0, 2*base_demand);

    auto base_uncertainty = std::make_unique<robust_model::UncertaintySet>(dummy_model);
    std::vector<robust_model::SOCExpression<robust_model::UncertaintyVariable>> transformation;
    robust_model::SOCExpression<robust_model::UncertaintyVariable> previous_uncertainty(base_demand);
    for (std::size_t t = 0; t < T; ++t) {
        auto const lastvar = base_uncertainty->add_variable(
                "base_uncertainty" + std::to_string(t), t, -nu, nu);
        transformation.emplace_back(previous_uncertainty + lastvar);
        previous_uncertainty += alpha * lastvar;
    }
    base_uncertainty->add_special_type_constraint(robust_model::UncertaintySet::SpecialSetType::BALL, nu);
    auto base_uncertainty_sampler = std::make_unique<robust_model::UniformL2BallRejectUncertaintySampler>(
            *base_uncertainty, nu, false);

    dummy_model.non_const_uncertainty_set().set_uncertainty_sampler(
            std::make_unique<robust_model::OwningTransformedUncertaintySampler>(
            dummy_model.uncertainty_set(),
            std::move(base_uncertainty),
            std::move(base_uncertainty_sampler),
            std::move(transformation)));

    return dummy_model.uncertainty_set().generate_uncertainty(size);
}

std::string test(Parameter const& parameter) {
    size_t const train_size = parameter.size;
    size_t const T = parameter.T;
    double const alpha = parameter.alpha;
    size_t const iteration = parameter.iteration;
    double const overage_cost = parameter.overage_cost;
    double const eoh_underage_cost = parameter.eoh_underage;
    std::vector<double> const radii = {0., .001, .01, .1, .3162, 1., 1.7783, 3.1623, 5.6234, 10., 17.783, 31.623};

    for (size_t i = 0; i < 5; ++i) {
        bool success = true;
        std::string results_string;
        std::string const parameter_string =
                std::string(parameter.cross_validation ? "CV" : "AV") + ";" +
                std::to_string(iteration) + "; " + std::to_string(train_size) + "; " +
                std::to_string(T) + "; " + std::to_string(alpha) + ";" +
                std::to_string(overage_cost) + ";" + std::to_string(eoh_underage_cost);
        auto const train_data = generate_data(train_size, T, alpha);
        auto const test_data = generate_data(10000, T, alpha);
        for (auto const mode: {data_models::DataDrivenLiftedInventoryManagementModel::Mode::AFFINE,
                               data_models::DataDrivenLiftedInventoryManagementModel::Mode::LIFTED_EQUIDISTANT}) {
            std::string const mode_string = data_models::DataDrivenLiftedInventoryManagementModel::to_string(mode);
            if (parameter.cross_validation) {
                data_models::CVModel<data_models::DataDrivenLiftedInventoryManagementModel> model
                        ({mode, overage_cost, eoh_underage_cost, 4}, radii, 5);
                if (not model.train(train_data)) {
                    success = false;
                    break;
                }
                auto const train_time = model.train_time();
                auto const train_score = model.test(train_data);
                auto const test_score = model.test(test_data);
                results_string += parameter_string + "; " + mode_string + "; "
                                  + std::to_string(model.best_radius()) + "; " + std::to_string(train_time) + "; "
                                  + train_score.csv_string() + "; " + test_score.csv_string() + "\n";
            } else {
                for (double const diam: radii) {
                    data_models::DataDrivenLiftedInventoryManagementModel model(mode, diam, overage_cost,
                                                                                eoh_underage_cost,
                                                                                4);
                    if (not model.train(train_data)) {
                        success = false;
                        break;
                    }
                    auto const train_time = model.train_time();
                    auto const train_score = model.test(train_data);
                    auto const test_score = model.test(test_data);
                    results_string += parameter_string + "; " + mode_string + "; "
                                      + std::to_string(diam) + "; " + std::to_string(train_time) + "; "
                                      + train_score.csv_string() + "; " + test_score.csv_string() + "\n";
                }
            }
        }
        if (success)
            return results_string;
        else
            helpers::warning_throw("No Solution found in iteration"+std::to_string(i));
    }
}


int main() {
    std::string run_name = "data_driven_inventory_revision_" + helpers::time_stamp();
    std::ofstream output_stream("../results/" + run_name + ".csv");
    helpers::global_logger.set_logfile("../logs/" + run_name + ".log");
    ParameterIterator iterator;
    data_models::ParallelDataInstanceEvaluator<Parameter> evaluator(iterator, test, output_stream);
    output_stream
            << "cross_validation; iteration; train_size; num_stages; alpha; overage_cost; eoh_underage; method; radius; train_time; train_mean; train_std; train_valid; test_mean; test_std; test_valid"
            << std::endl;
    evaluator.run_tests(6);
    helpers::global_logger << "END OF TESTS!";
    return 0;
}