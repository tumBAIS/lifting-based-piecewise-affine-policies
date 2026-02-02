#ifndef PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_UNCERTAINTYSOLVER_H
#define PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_UNCERTAINTYSOLVER_H

#include "../../models/uncertainty/UncertaintySet.h"
#include "../soc_solvers/SOCSolverBase.h"
#include "../soc_solvers/GurobiSOCSolver.h"
#include "../../models/SOCModel.h"

namespace robust_model {

class UncertaintySolver : public solvers::SolverBase {
public:
    using Objective = ObjectiveBase<AffineExpression<UncertaintyVariable::Reference>>;
public:
    explicit UncertaintySolver(UncertaintySet const& uncertainty_set);

    void set_objective(Objective const& objective);

    UncertaintyRealization realization() const;

private:

    void build_implementation() override;

    void solve_implementation() override;

    SOCModel& soc_model();

    UncertaintySet const& uncertainty_set() const;

    solvers::SOCSolverBase& soc_solver();

    std::vector<SOCVariable::Reference> const& uvar_soc_references() const;

private:
    UncertaintySet const& _uncertainty_set;
    SOCModel _soc_model;
    std::unique_ptr<solvers::GurobiSOCSolver> _soc_solver;
    std::vector<SOCVariable::Reference> _uvar_soc_references;
};

}

#endif //PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_UNCERTAINTYSOLVER_H
