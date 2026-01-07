#include "UncertaintySolver.h"

namespace robust_model {

UncertaintySolver::UncertaintySolver(robust_model::UncertaintySet const& uncertainty_set) :
        _uncertainty_set(uncertainty_set),
        _soc_model("UncertaintySet"),
        _soc_solver(std::make_unique<solvers::GurobiSOCSolver>(soc_model()))
        {}


void UncertaintySolver::set_objective(UncertaintySolver::Objective const& objective) {
    soc_model().clear_and_set_objective(
            {objective.sense(),
             objective.expression().translate_to_other(uvar_soc_references())});
    soc_solver().objectives_reset();
}

UncertaintyRealization UncertaintySolver::realization() const {
    std::vector<double> values;
    values.reserve(uncertainty_set().num_variables());
    for(auto const& soc_uvar : uvar_soc_references()){
        values.emplace_back(soc_uvar->solution());
    }
    return UncertaintyRealization(std::move(values));
}

void UncertaintySolver::build_implementation() {
    for(auto const& uvar : uncertainty_set().variables()){
        _uvar_soc_references.emplace_back(soc_model().add_variable(uvar.name(), uvar.lb(), uvar.ub()));
    }
    for(auto const& constraint : uncertainty_set().uncertainty_constraints()){
        soc_model().add_constraint(constraint.translate_to_other(uvar_soc_references()));
    }
}

void UncertaintySolver::solve_implementation() {
    set_parameters_to_other(soc_solver());
    soc_solver().solve();
    set_results_from_other(soc_solver());
}

SOCModel& UncertaintySolver::soc_model() {
    return _soc_model;
}

UncertaintySet const& UncertaintySolver::uncertainty_set() const {
    return _uncertainty_set;
}

solvers::SOCSolverBase& UncertaintySolver::soc_solver() {
    return *_soc_solver;
}

std::vector<SOCVariable::Reference> const& UncertaintySolver::uvar_soc_references() const {
    return _uvar_soc_references;
}

}