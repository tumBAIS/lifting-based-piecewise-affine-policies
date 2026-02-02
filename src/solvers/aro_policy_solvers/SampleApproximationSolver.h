#ifndef PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_SAMPLEAPPROXIMATIONSOLVER_H
#define PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_SAMPLEAPPROXIMATIONSOLVER_H

#include "AROPolicySolverBase.h"
#include "../../models/SOCModel.h"
#include "../soc_solvers/GurobiSOCSolver.h"

namespace robust_model {

class SampleApproximationSolver : public solvers::AROPolicySolverBase {
public:
    SampleApproximationSolver(
            ROModel const& model, size_t num_samples);

    SampleApproximationSolver(
            ROModel const& model,
            std::vector<std::vector<double>> sample_realizations);

    SampleApproximationSolver(
            ROModel const& model,
            std::vector<std::vector<double>> sample_realizations,
            size_t num_objective_relevant_samples
            );

    SolutionRealization specific_solution(std::vector<double> const& uncertainty_realization) const override;

private:
    void build_implementation() override;

    void solve_implementation() override;

    void add_variables();

    void add_constraints();

    void add_objective();

    SOCModel& soc_model();
    solvers::SOCSolverBase& soc_solver();

    size_t num_samples() const;

    size_t num_objective_relevant_samples() const;

    std::vector<double> const& sample_realization(size_t sample_id) const;

    std::vector<SOCVariable::Reference> const& sample_decisions(size_t sample_id) const;


    std::vector<std::vector<size_t>> non_anticipative_realization_groups(DecisionVariable::Index const& decision) const;

private:
    SOCModel _soc_model;
    std::unique_ptr<solvers::GurobiSOCSolver> _soc_solver;

    size_t const _num_objective_relevant_samples;
    std::vector<std::vector<double>> const _sample_realizations;
    std::vector<std::vector<SOCVariable::Reference>> _sample_decisions;
};

}

#endif //PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_SAMPLEAPPROXIMATIONSOLVER_H
