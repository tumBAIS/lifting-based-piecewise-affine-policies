#include "AffineAdjustablePolicyDualSolver.h"
#include "AffineAdjustablePolicySolver.h"

namespace robust_model {
AffineAdjustablePolicyDualSolver::AffineAdjustablePolicyDualSolver(ROModel const& model) :
        AROPolicySolverBase(model), _soc_model("Dual AARC of " + model.name()) {}

void AffineAdjustablePolicyDualSolver::add_uncertainty_constraint(
        UncertaintySet::Constraint const& constraint,
        UncertaintySetConstraintsSet::Index uncertainty_constraints_set_id,
        size_t ro_constraint_id) {
    add_uncertainty_constraint(constraint,
                               scaled_uncertainty_variables(ro_constraint_id, uncertainty_constraints_set_id),
                               "_constr" + std::to_string(ro_constraint_id) +
                               "_set" + std::to_string(uncertainty_constraints_set_id.raw_id())
    );
}


void AffineAdjustablePolicyDualSolver::build_implementation() {
    _soc_solver = std::make_unique<solvers::GurobiSOCSolver>(soc_model());

    AffineExpression<SOCVariable::Reference> dual_objective;
    std::vector<std::vector<AffineExpression<SOCVariable::Reference>>> adjustable_factors_duals;
    for (auto const& dvar: model().decision_variables()) {
        adjustable_factors_duals.emplace_back(dvar.num_dependencies());
    }
    std::vector<AffineExpression<SOCVariable::Reference>> adjustable_constants_duals(model().num_dvars());
    for (auto const& ro_constr: model().constraints()) {
        switch (ro_constr.expression().uncertainty_behaviour()) {
            case RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE:
            case RoAffineExpression::UncertaintyBehaviour::MULTI_UNION:
                _scaled_uncertainty_variables.emplace_back(add_dual_of_ro_constraint(
                        dual_objective, adjustable_factors_duals, adjustable_constants_duals, ro_constr));
                break;
            case RoAffineExpression::UncertaintyBehaviour::STOCHASTIC:
                add_dual_of_so_constraint(dual_objective, adjustable_factors_duals, adjustable_constants_duals, ro_constr);
                _scaled_uncertainty_variables.emplace_back();
        }
    }

    for (auto const& var: model().decision_variables()) {
        if (var.lb() != NO_VARIABLE_LB) {
            add_dual_of_ro_constraint(dual_objective, adjustable_factors_duals, adjustable_constants_duals,
                                      {var.reference() >= var.lb(), "LB_" + var.name()});
        }
        if (var.ub() != NO_VARIABLE_UB) {
            add_dual_of_ro_constraint(dual_objective, adjustable_factors_duals, adjustable_constants_duals,
                                      {var.reference() <= var.ub(), "UB_" + var.name()});
        }
    }


    helpers::exception_check(model().objective().sense() == ObjectiveSense::MIN, "Only minimization implemented!");
    switch (model().objective().expression().uncertainty_behaviour()) {
        case RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE:
        case RoAffineExpression::UncertaintyBehaviour::MULTI_UNION: {
            auto const scaled_uncertainties = add_dual_of_ro_constraint(
                    dual_objective, adjustable_factors_duals,
                    adjustable_constants_duals,
                    {model().objective().expression() <= 0, "epi_objective"}
            );
            AffineExpression<SOCVariable::Reference> epi_dual;
            for (auto const& scaled_uncertainty: scaled_uncertainties) {
                epi_dual += scaled_uncertainty.scale;
            }
            soc_model().add_constraint(epi_dual == 1, "epi_dual");
        }
            break;
        case RoAffineExpression::UncertaintyBehaviour::STOCHASTIC:{
            SOCVariable::Reference epi_dual_var = add_dual_of_so_constraint(
                    dual_objective, adjustable_factors_duals, adjustable_constants_duals,
                    {model().objective().expression() <= 0, "epi_objective"});
            soc_model().add_constraint(epi_dual_var == 1, "epi_dual");
        }
    }

    soc_model().add_objective({ObjectiveSense::MAX, dual_objective});

    for (auto const& dvar: model().decision_variables()) {
        for (auto const& dependency: dvar.dependencies()) {
            auto const& dual_term = adjustable_factors_duals.at(dvar.id().raw_id()).at(dependency.id().raw_id());
            soc_model().add_constraint(
                    dual_term == 0, "DualConstr" + dvar.name() + dependency.uncertainty_variable()->name());
        }
        auto const& dual_term = adjustable_constants_duals.at(dvar.id().raw_id());
        soc_model().add_constraint(dual_term == 0, "DualConstr" + dvar.name() + "constant");
    }
}


void AffineAdjustablePolicyDualSolver::solve_implementation() {
    set_parameters_to_other(soc_solver());
    soc_solver().solve();
    set_results_from_other(soc_solver());
}

std::vector<AffineAdjustablePolicyDualSolver::ScaledUncertainties>
AffineAdjustablePolicyDualSolver::add_dual_of_ro_constraint(
        AffineExpression<SOCVariable::Reference>& dual_objective,
        std::vector<std::vector<AffineExpression<SOCVariable::Reference>>>& adjustable_factors_duals,
        std::vector<AffineExpression<SOCVariable::Reference>>& adjustable_constants_duals,
        ROModel::RoConstraint const& ro_constr) {

    if (ro_constr.sense() == ConstraintSense::EQ) {
        auto const scaled_uncertainty = generate_scaled_uncertainty(ro_constr.name());
        add_dual_of_ro_expression(
                dual_objective, adjustable_factors_duals, adjustable_constants_duals, ro_constr.expression(),
                scaled_uncertainty,
                ro_constr.name());
        return {};
    } else {
        std::vector<ScaledUncertainties> scaled_uncertainties;

        auto const leq_expr = (ro_constr.sense() == ConstraintSense::LEQ ? 1. : -1.) * ro_constr.expression();
        auto const multi_scale =
                (leq_expr.uncertainty_behaviour() == RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE) ?
                std::optional<SOCVariable::Index>(soc_model().add_variable("multi_scale_" + ro_constr.name())) :
                std::optional<SOCVariable::Index>{};
        for (auto const& uncertainty_constraint_set: model().uncertainty_set().constraint_sets()) {
            scaled_uncertainties.emplace_back(
                    generate_scaled_uncertainty(ro_constr.name(), {uncertainty_constraint_set}, multi_scale));
            add_dual_of_ro_expression(
                    dual_objective, adjustable_factors_duals, adjustable_constants_duals, leq_expr,
                    scaled_uncertainties.back(),
                    ro_constr.name() + "_USet" + std::to_string(uncertainty_constraint_set.raw_id()));
        }
        return scaled_uncertainties;
    }
}

void
AffineAdjustablePolicyDualSolver::add_dual_of_ro_expression(
        AffineExpression<SOCVariable::Reference>& dual_objective,
        std::vector<std::vector<AffineExpression<SOCVariable::Reference>>>& adjustable_factors_duals,
        std::vector<AffineExpression<SOCVariable::Reference>>& adjustable_constants_duals,
        RoAffineExpression const& ro_expression,
        ScaledUncertainties const& scaled_uncertainty,
        std::string const& name_addendum) {
    helpers::exception_check(
            ro_expression.uncertainty_behaviour() == RoAffineExpression::UncertaintyBehaviour::MULTI_UNION or
            ro_expression.uncertainty_behaviour() == RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE,
            "Dual of RO Expression only implemented for UNION and AVERAGE behaviour!"
    );
    auto const& soc_uvars = scaled_uncertainty.uncertainty_variables;
    auto const& soc_scale = scaled_uncertainty.scale;
    for (auto const& scaled_dvar: ro_expression.decisions().scaled_variables()) {
        for (auto const& dependency: scaled_dvar.variable()->dependencies()) {
            adjustable_factors_duals.at(scaled_dvar.variable().raw_id()).at(dependency.id().raw_id())
                    += scaled_dvar.scale() * soc_uvars.at(dependency.uncertainty_variable().raw_id());
        }
        adjustable_constants_duals.at(scaled_dvar.variable().raw_id())
                += scaled_dvar.scale() * soc_scale;
    }
    for (auto const& scaled_uvar: ro_expression.uncertainties().scaled_variables()) {
        dual_objective += scaled_uvar.scale() * soc_uvars.at(scaled_uvar.variable().raw_id());
    }
    dual_objective += ro_expression.constant() * soc_scale;
    for (auto const& scaled_udvar: ro_expression.uncertainty_decisions().scaled_variables()) {
        helpers::exception_check(scaled_udvar.variable().decision_variable()->dependencies().empty(),
                                 "Uncertainty Scaled Variables are not supported for recourse decisions");
        adjustable_constants_duals.at(scaled_udvar.variable().decision_variable().raw_id()) +=
                scaled_udvar.scale() * soc_uvars.at(scaled_udvar.variable().uncertainty_variable().raw_id());
    }
}

SOCVariable::Reference AffineAdjustablePolicyDualSolver::add_dual_of_so_constraint(
        AffineExpression<SOCVariable::Reference>& dual_objective,
        std::vector<std::vector<AffineExpression<SOCVariable::Reference>>>& adjustable_factors_duals,
        std::vector<AffineExpression<SOCVariable::Reference>>& adjustable_constants_duals,
        ROModel::RoConstraint const& so_constr) {
    helpers::exception_check(
            so_constr.expression().uncertainty_behaviour() == RoAffineExpression::UncertaintyBehaviour::STOCHASTIC,
            "Dual of SO Expression only implemented for STOCHASTIC behaviour!"
    );
    helpers::exception_check(
            so_constr.sense() != ConstraintSense::EQ,
            "Dual of SO for inequalities behaviour!"
    );

    auto const leq_expr = (so_constr.sense() == ConstraintSense::LEQ ? 1. : -1.) * so_constr.expression();

    auto const expectations_helper = AffinePolicyExpectationHelper(model());
    auto const adjustable_factors_scales = expectations_helper.expected_adjustable_factor_scales(leq_expr);
    auto const adjustable_constants_scales = expectations_helper.expected_adjustable_constant_scales(leq_expr);
    auto const expected_constant = expectations_helper.expected_constant(leq_expr);

    SOCVariable::Reference constr_dual_variable = soc_model().add_variable("dualVar_" + so_constr.name(), 0.);

    for (auto const& dvar: model().decision_variables()) {
        for (auto const& dependency: dvar.dependencies()) {
            auto const scale = adjustable_factors_scales.at(dvar.id().raw_id()).at(dependency.id().raw_id());
            if (scale != 0)
                adjustable_factors_duals.at(dvar.id().raw_id()).at(dependency.id().raw_id()) +=
                        constr_dual_variable * scale;
        }
        auto const scale = adjustable_constants_scales.at(dvar.id().raw_id());
        if (scale != 0)
            adjustable_constants_duals.at(dvar.id().raw_id()) += constr_dual_variable * scale;
    }
    dual_objective += constr_dual_variable * expected_constant;

    return constr_dual_variable;
}

AffineAdjustablePolicyDualSolver::ScaledUncertainties
AffineAdjustablePolicyDualSolver::generate_scaled_uncertainty(
        std::string const& name_addendum,
        std::optional<UncertaintySetConstraintsSet::Index> uncertainty_constraint_set,
        std::optional<SOCVariable::Index> multi_scale
) {
    std::string name_addendum_uset =
            name_addendum +
            ((uncertainty_constraint_set.has_value())
             ? ("_USet" + std::to_string(
                            uncertainty_constraint_set.value().raw_id()))
             : "");

    std::vector<SOCVariable::Reference> uncertainty_variables;
    for (auto const& uvar: model().uncertainty_variables()) {
        uncertainty_variables.emplace_back(soc_model().add_variable(
                uvar.name() + name_addendum_uset));
    }
    auto scaled_uncertainty = ScaledUncertainties{uncertainty_variables,
                                                  multi_scale.has_value() ?
                                                  multi_scale.value()->reference() :
                                                  soc_model().add_variable(
                                                          "scale" + name_addendum_uset,
                                                          uncertainty_constraint_set.has_value() ? 0. : NO_VARIABLE_LB)
    };
    if (uncertainty_constraint_set.has_value())
        add_uncertainty_constraints(scaled_uncertainty, uncertainty_constraint_set.value(), name_addendum);
    return scaled_uncertainty;
}

void AffineAdjustablePolicyDualSolver::add_uncertainty_constraints(
        ScaledUncertainties const& scaled_uncertainty,
        UncertaintySetConstraintsSet::Index const& uncertainty_constraint_set,
        std::string const& name_addendum) {
    auto const& soc_uvars = scaled_uncertainty.uncertainty_variables;
    auto const& soc_scale = scaled_uncertainty.scale;
    for (auto const& uvar: model().uncertainty_variables()) {
        soc_model().add_constraint(
                soc_uvars.at(uvar.reference().raw_id()) >= uvar.lb() * soc_scale,
                uvar.name() + name_addendum + "_USet" + std::to_string(uncertainty_constraint_set.raw_id()) +
                "_scaledlb");
        soc_model().add_constraint(
                soc_uvars.at(uvar.reference().raw_id()) <= uvar.ub() * soc_scale,
                uvar.name() + name_addendum + "_USet" + std::to_string(uncertainty_constraint_set.raw_id()) +
                "_scaledub");
    }
    for (auto const& uconstr: uncertainty_constraint_set->constraints()) {
        add_uncertainty_constraint(
                uconstr, scaled_uncertainty,
                name_addendum + "_USet" + std::to_string(uncertainty_constraint_set.raw_id()));
    }
}


void AffineAdjustablePolicyDualSolver::add_uncertainty_constraint(
        UncertaintySet::Constraint const& constraint,
        ScaledUncertainties const& scaled_uncertainty,
        std::string const& name_addendum) {
    auto const translate_aff_expr = [&](AffineExpression<UncertaintyVariable::Reference> const& aff_expr) {
        auto translated_aff_expr = aff_expr.translate_to_other(scaled_uncertainty.uncertainty_variables);
        translated_aff_expr += aff_expr.constant() * scaled_uncertainty.scale;
        translated_aff_expr -= aff_expr.constant();
        return translated_aff_expr;
    };
    if (constraint.expression().is_affine()) {
        soc_model().add_constraint(
                SOCConstraint<SOCVariable>(
                        constraint.sense(),
                        translate_aff_expr(constraint.expression().affine()),
                        constraint.name() + name_addendum));
    } else {
        std::vector<AffineExpression<SOCVariable::Reference>> new_normed_vector;
        for (auto const& aff_expr: constraint.expression().normed_vector().normed_vector()) {
            new_normed_vector.emplace_back(translate_aff_expr(aff_expr));
        }
        soc_model().add_constraint(
                SOCConstraint<SOCVariable>(
                        constraint.sense(),
                        SOCExpression<SOCVariable>(
                                NormedAffineVector<SOCVariable>(
                                        constraint.expression().normed_vector().norm_type(), new_normed_vector),
                                translate_aff_expr(constraint.expression().affine())),
                        constraint.name() + name_addendum));
    }
}

SOCModel& AffineAdjustablePolicyDualSolver::soc_model() {
    return _soc_model;
}

solvers::SOCSolverBase& AffineAdjustablePolicyDualSolver::soc_solver() {
    return *_soc_solver;
}

AffineAdjustablePolicyDualSolver::ScaledUncertainties const&
AffineAdjustablePolicyDualSolver::scaled_uncertainty_variables(
        size_t ro_constraint_id, UncertaintySetConstraintsSet::Index uncertainty_constraints_set_id) const {
    return _scaled_uncertainty_variables.at(ro_constraint_id).at(
            uncertainty_constraints_set_id.raw_id());
}

SolutionRealization
AffineAdjustablePolicyDualSolver::specific_solution(std::vector<double> const& uncertainty_realization) const {
    helpers::exception_throw("Not implemented!");
    return {model(), {}, {}};
}


}
