#ifndef PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_AFFINEADJUSTABLEPOLICYDUALSOLVER_H
#define PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_AFFINEADJUSTABLEPOLICYDUALSOLVER_H

#include "../../models/ROModel.h"
#include "../../models/SOCModel.h"
#include "../../helpers/helpers.h"
#include "AROPolicySolverBase.h"
#include "../soc_solvers/GurobiSOCSolver.h"

namespace robust_model {

class AffineAdjustablePolicyDualSolver : public solvers::AROPolicySolverBase {
public:
    explicit AffineAdjustablePolicyDualSolver(ROModel const& model);

    void add_uncertainty_constraint(
            UncertaintySet::Constraint const& constraint,
            UncertaintySetConstraintsSet::Index uncertainty_constraints_set_id,
            size_t ro_constraint_id);

    SolutionRealization specific_solution(std::vector<double> const& uncertainty_realization) const override;

    struct ScaledUncertainties {
        std::vector<SOCVariable::Reference> uncertainty_variables;
        SOCVariable::Reference scale;

        double value(UncertaintyVariable::Index uvar) const{
            return uncertainty_variables.at(uvar.raw_id())->solution()/scale->solution();
        }
    };

    ScaledUncertainties const& scaled_uncertainty_variables(
            size_t ro_constraint_id,
            UncertaintySetConstraintsSet::Index uncertainty_constraints_set_id) const;

private:
    void build_implementation() override;

    void solve_implementation() override;

    std::vector<ScaledUncertainties> add_dual_of_ro_constraint(
            AffineExpression<SOCVariable::Reference>& dual_objective,
            std::vector<std::vector<AffineExpression<SOCVariable::Reference>>>& adjustable_factors_duals,
            std::vector<AffineExpression<SOCVariable::Reference>>& adjustable_constants_duals,
            ROModel::RoConstraint const& ro_constr);

    void add_dual_of_ro_expression(
            AffineExpression<SOCVariable::Reference>& dual_objective,
            std::vector<std::vector<AffineExpression<SOCVariable::Reference>>>& adjustable_factors_duals,
            std::vector<AffineExpression<SOCVariable::Reference>>& adjustable_constants_duals,
            RoAffineExpression const& ro_expression,
            ScaledUncertainties const& scaled_uncertainty,
            std::string const& name_addendum
    );

    SOCVariable::Reference add_dual_of_so_constraint(
            AffineExpression<SOCVariable::Reference>& dual_objective,
            std::vector<std::vector<AffineExpression<SOCVariable::Reference>>>& adjustable_factors_duals,
            std::vector<AffineExpression<SOCVariable::Reference>>& adjustable_constants_duals,
            ROModel::RoConstraint const& so_constr);

    ScaledUncertainties
    generate_scaled_uncertainty(std::string const& name_addendum,
                                std::optional<UncertaintySetConstraintsSet::Index> uncertainty_constraint_set = {},
                                std::optional<SOCVariable::Index> multi_scale = {}
    );

    void add_uncertainty_constraints(
            ScaledUncertainties const& scaled_uncertainty,
            UncertaintySetConstraintsSet::Index const& uncertainty_constraint_set,
            std::string const& name_addendum);

    void add_uncertainty_constraint(
            UncertaintySet::Constraint const& constraint,
            ScaledUncertainties const& scaled_uncertainty,
            std::string const& name_addendum);

    SOCModel& soc_model();

    solvers::SOCSolverBase& soc_solver();


private:
    SOCModel _soc_model;
    std::unique_ptr<solvers::GurobiSOCSolver> _soc_solver;
    std::vector<std::vector<ScaledUncertainties>> _scaled_uncertainty_variables;
};

}

#endif //PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_AFFINEADJUSTABLEPOLICYDUALSOLVER_H
