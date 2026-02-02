#include "SampleApproximationSolver.h"

namespace robust_model {

SampleApproximationSolver::SampleApproximationSolver(
        ROModel const& model,
        size_t num_samples) :
        SampleApproximationSolver(model, model.uncertainty_set().generate_uncertainty(num_samples)) {}

SampleApproximationSolver::SampleApproximationSolver(
        ROModel const& model,
        std::vector<std::vector<double>> sample_realizations)
        : SampleApproximationSolver(model, std::move(sample_realizations), sample_realizations.size()) {}

SampleApproximationSolver::SampleApproximationSolver(
        ROModel const& model,
        std::vector<std::vector<double>> sample_realizations,
        size_t num_objective_relevant_samples) :
        AROPolicySolverBase(model),
        _soc_model("Sample Approximation of " + model.name()),
        _sample_realizations(std::move(sample_realizations)),
        _num_objective_relevant_samples(num_objective_relevant_samples) {}


void SampleApproximationSolver::build_implementation() {
    add_variables();
    add_constraints();
    add_objective();
    _soc_solver = std::make_unique<solvers::GurobiSOCSolver>(soc_model());
}

void SampleApproximationSolver::solve_implementation() {
    set_parameters_to_other(soc_solver());
    soc_solver().solve();
    set_results_from_other(soc_solver());
}

void SampleApproximationSolver::add_variables() {
    helpers::exception_check(_sample_decisions.empty(), "Only add variables once!");
    _sample_decisions.resize(num_samples());
    for (auto const& dvar: model().decision_variables()) {
        for (auto const& group: non_anticipative_realization_groups(dvar.id())) {
            auto const soc_var = soc_model().add_variable(dvar.name() + "_S" + std::to_string(group.front()), dvar.lb(),
                                                          dvar.ub());
            for (auto i: group) {
                _sample_decisions.at(i).emplace_back(soc_var);
            }
        }
    }
}

void SampleApproximationSolver::add_constraints() {
    for (auto const& constr: model().constraints()) {
        for (size_t i = 0; i < num_samples(); ++i) {
            soc_model().add_constraint(
                    {constr.sense(),
                     constr.expression().substitute_to_other_affine<SOCVariable::Reference>(
                             sample_decisions(i), sample_realization(i)),
                     constr.name() + "_S" + std::to_string(i)
                    }
            );
        }
    }
}

void SampleApproximationSolver::add_objective() {
    auto const epi_var = soc_model().add_variable("OBJ EPI");
    double const sense_scale = (model().objective().sense() == ObjectiveSense::MAX) ? -1. : 1.;

    soc_model().add_objective({model().objective().sense(), sense_scale * epi_var + 0.});

    switch (model().objective().expression().uncertainty_behaviour()) {
        case RoAffineExpression::UncertaintyBehaviour::MULTI_UNION:
        case RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE: {
            for (size_t i = 0; i < num_objective_relevant_samples(); ++i) {
                soc_model().add_constraint(
                        {
                                ConstraintSense::GEQ,
                                epi_var - sense_scale *
                                          model().objective().expression().substitute_to_other_affine<SOCVariable::Reference>(
                                                  sample_decisions(i), sample_realization(i)),
                                "OBJ_EPI_S" + std::to_string(i)
                        });
            }
        }
            break;
        case RoAffineExpression::UncertaintyBehaviour::STOCHASTIC: {
            AffineExpression<SOCVariable::Reference> average_objective;
            for (size_t i = 0; i < num_objective_relevant_samples(); ++i) {
                average_objective +=
                        model().objective().expression().substitute_to_other_affine<SOCVariable::Reference>(
                                sample_decisions(i), sample_realization(i));
            }
            soc_model().add_constraint(
                    {
                            ConstraintSense::GEQ,
                            epi_var - average_objective * (sense_scale / double(num_objective_relevant_samples())),
                            "OBJ_EPI"
                    });
        }
            break;
    }
}

SOCModel& SampleApproximationSolver::soc_model() {
    return _soc_model;
}

solvers::SOCSolverBase& SampleApproximationSolver::soc_solver() {
    return *_soc_solver;
}

size_t SampleApproximationSolver::num_samples() const {
    return _sample_realizations.size();
}

size_t SampleApproximationSolver::num_objective_relevant_samples() const {
    return _num_objective_relevant_samples;
}

std::vector<double> const& SampleApproximationSolver::sample_realization(size_t sample_id) const {
    return _sample_realizations.at(sample_id);
}

std::vector<SOCVariable::Reference> const& SampleApproximationSolver::sample_decisions(size_t sample_id) const {
    return _sample_decisions.at(sample_id);
}

SolutionRealization
SampleApproximationSolver::specific_solution(std::vector<double> const& uncertainty_realization) const {
    std::vector<double> decisions(model().num_dvars());
    for (auto const& dvar: model().decision_variables()) {
        if (dvar.dependencies().empty()) {
            decisions.at(dvar.id().raw_id()) = _sample_decisions.front().at(dvar.id().raw_id())->solution();
        } else {
            decisions.at(dvar.id().raw_id()) = std::nan("adjustable");
        }
    }
    return {model(), uncertainty_realization, decisions};
}

std::vector<std::vector<size_t>>
SampleApproximationSolver::non_anticipative_realization_groups(DecisionVariable::Index const& decision) const {
    std::map<std::vector<double>, std::vector<size_t>> groups;
    std::vector<double> dependency_realization(decision->dependencies().size());
    for (size_t i = 0; i < num_samples(); ++i) {
        for (auto const& dependency: decision->dependencies()) {
            dependency_realization[dependency.id().raw_id()] =
                    sample_realization(i)[dependency.uncertainty_variable()->id().raw_id()];
        }
        groups[dependency_realization].push_back(i);
    }
    std::vector<std::vector<size_t>> groups_vector;
    for (const auto& [realization, group]: groups) {
        groups_vector.emplace_back(group);
    }
    return groups_vector;
}


}