#include "AffineAdjustablePolicySolver.h"
#include "../../helpers/helpers.h"

namespace robust_model {

AffineAdjustablePolicySolver::AffineAdjustablePolicySolver(ROModel const& model) :
        solvers::AROPolicySolverBase(model),
        _soc_model("AARC of " + model.name()) {}


void AffineAdjustablePolicySolver::add_ro_constraint(ROModel::RoConstraint const& ro_constr) {
    switch (ro_constr.sense()) {
        case ConstraintSense::GEQ: {
            auto dual_objective = add_counterpart_constraints_for_minimization(ro_constr.expression(),
                                                                               ro_constr.name());
            soc_model().add_constraint(dual_objective >= 0, ro_constr.name() + "RC");
            return;
        }
        case ConstraintSense::LEQ: {
            auto dual_objective = add_counterpart_constraints_for_minimization(-ro_constr.expression(),
                                                                               ro_constr.name());
            soc_model().add_constraint(dual_objective >= 0, ro_constr.name() + "RC");
            return;
        }
        case ConstraintSense::EQ: {
            add_rc_constraints_for_equality(ro_constr.expression(), ro_constr.name());
            return;
        }
        default:
            helpers::exception_throw("Case not implemented!!!");
    }
}

AffineExpression<SOCVariable::Reference>
AffineAdjustablePolicySolver::add_counterpart_constraints_for_minimization(RoAffineExpression const& expr,
                                                                           std::string const& name_addendum) {
    switch (expr.uncertainty_behaviour()) {
        case RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE:
        case RoAffineExpression::UncertaintyBehaviour::MULTI_UNION:
            return add_robust_counterpart_constraints_for_minimization(expr, name_addendum);
        case RoAffineExpression::UncertaintyBehaviour::STOCHASTIC:
            return add_stochastic_counterpart_constraints_for_minimization(expr, name_addendum);
    }
}

AffineExpression<SOCVariable::Reference>
AffineAdjustablePolicySolver::add_robust_counterpart_constraints_for_minimization(RoAffineExpression const& expr,
                                                                                  std::string const& name_addendum) {
    auto const epigraph_var = soc_model().add_variable("EpiVar" + name_addendum);
    AffineExpression<SOCVariable::Reference> average; // this is only needed for average and not union behaviour!
    for (auto const uncertainty_union_set: model().uncertainty_set().constraint_sets()) {
        AffineExpression<SOCVariable::Reference> dual_objective;
        std::vector<AffineExpression<SOCVariable::Reference>> dual_constraint_expressions(model().num_uvars());

        add_dual_of_uncertainty_set(dual_objective, dual_constraint_expressions, uncertainty_union_set, name_addendum);
        add_dual_of_expression(dual_objective, dual_constraint_expressions, expr);


        for (auto const& var: model().uncertainty_variables()) {
            soc_model().add_constraint(dual_constraint_expressions[var.id().raw_id()] == 0,
                                       name_addendum + "_DualConstr_" + var.name());
        }
        if (expr.uncertainty_behaviour() == RoAffineExpression::UncertaintyBehaviour::MULTI_UNION) {
            soc_model().add_constraint(epigraph_var <= dual_objective,
                                       "EpiConstr" + name_addendum + "_US" +
                                       std::to_string(uncertainty_union_set.raw_id()));
        }
        if (expr.uncertainty_behaviour() == RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE) {
            average += dual_objective;
        }
    }
    if (expr.uncertainty_behaviour() == RoAffineExpression::UncertaintyBehaviour::MULTI_AVERAGE) {
        average /= double(model().uncertainty_set().constraint_sets().size());
        soc_model().add_constraint(epigraph_var <= average,
                                   "EpiConstr" + name_addendum + "_AVG");
    }
    return AffineExpression<SOCVariable::Reference>(epigraph_var);
}

AffineExpression<SOCVariable::Reference>
AffineAdjustablePolicySolver::add_stochastic_counterpart_constraints_for_minimization(RoAffineExpression const& expr,
                                                                                      std::string const& name_addendum) {
    auto const expectations_helper = AffinePolicyExpectationHelper(model());
    auto const adjustable_factors_scales = expectations_helper.expected_adjustable_factor_scales(expr);
    auto const adjustable_constants_scales = expectations_helper.expected_adjustable_constant_scales(expr);
    auto const expected_constant = expectations_helper.expected_constant(expr);

    AffineExpression<SOCVariable::Reference> res_expr(expected_constant);
    for (auto const& dvar: model().decision_variables()) {
        for (auto const& dependency: dvar.dependencies()) {
            if (adjustable_factors_scales.at(dvar.id().raw_id()).at(dependency.id().raw_id()) != 0)
                res_expr += adjustable_factors_scales.at(dvar.id().raw_id()).at(dependency.id().raw_id()) *
                            adjustable_factor(dependency);
        }
        if (adjustable_constants_scales.at(dvar.id().raw_id()) != 0)
            res_expr += adjustable_constants_scales.at(dvar.id().raw_id()) *
                        adjustable_constant(dvar.id());
    }
    return res_expr;
}

void AffineAdjustablePolicySolver::add_rc_constraints_for_equality(RoAffineExpression const& expr,
                                                                   std::string const& name_addendum) {
    helpers::exception_check(expr.uncertainty_behaviour() != RoAffineExpression::UncertaintyBehaviour::STOCHASTIC,
                             "Equality not yet allowed in stochastic case!"
    );
    AffineExpression<SOCVariable::Reference> adjustable_constants_equation(expr.constant());
    std::vector<AffineExpression<SOCVariable::Reference>> adjustable_factor_equations(model().num_uvars());
    for (auto const& suvar: expr.uncertainties().scaled_variables()) {
        adjustable_factor_equations[suvar.variable().raw_id()] += suvar.scale();
    }
    for (auto const& sdvar: expr.decisions().scaled_variables()) {
        adjustable_constants_equation += sdvar.scale() * adjustable_constant(sdvar.variable());
        for (auto const& dependency: sdvar.variable()->dependencies()) {
            adjustable_factor_equations[dependency.uncertainty_variable().raw_id()] +=
                    sdvar.scale() * adjustable_factor(dependency);
        }
    }
    for (auto const& usdvar: expr.uncertainty_decisions().scaled_variables()) {
        helpers::exception_check(usdvar.variable().decision_variable()->dependencies().empty(),
                                 "Uncertainty Scaled Variables are not supported for recourse decisions");
        adjustable_factor_equations[usdvar.variable().decision_variable().raw_id()] +=
                usdvar.scale() * adjustable_constant(usdvar.variable().decision_variable());
    }
    for (auto const& var: model().uncertainty_variables()) {
        soc_model().add_constraint(adjustable_factor_equations[var.id().raw_id()] == 0,
                                   name_addendum + "_AffRC_" + var.name());
    }
    soc_model().add_constraint(adjustable_constants_equation == 0, name_addendum + "_AffRC_" + "Base");
}


void
AffineAdjustablePolicySolver::add_dual_of_uncertainty_set(AffineExpression<SOCVariable::Reference>& dual_objective,
                                                          std::vector<AffineExpression<SOCVariable::Reference>>& dual_constraint_expressions,
                                                          UncertaintySetConstraintsSet::Index const union_set_constraints,
                                                          std::string const& name_addendum) {
    for (auto const& constr: model().uncertainty_set().uncertainty_constraints(union_set_constraints)) {
        auto const dv = soc_model().add_variable(name_addendum + "_" + constr.name() + "_DVar", constr.dual_lb(),
                                                 constr.dual_ub());
        dual_objective += constr.soc_expression().affine().constant() * dv;
        for (auto const& svar: constr.soc_expression().affine().linear().scaled_variables()) {
            dual_constraint_expressions[svar.variable().raw_id()] += svar.scale() * dv;
        }
        if (not constr.soc_expression().is_affine()) {
            add_dual_of_normed_vector(constr.soc_expression().normed_vector(), dv,
                                      dual_objective, dual_constraint_expressions,
                                      name_addendum + constr.name());
        }
    }

    add_dual_of_uvar_bounds(dual_objective, dual_constraint_expressions, name_addendum);
}

void
AffineAdjustablePolicySolver::add_dual_of_normed_vector(const NormedAffineVector<UncertaintyVariable>& normed_vector,
                                                        SOCVariable::Reference dv,
                                                        AffineExpression<SOCVariable::Reference>& dual_objective,
                                                        std::vector<AffineExpression<SOCVariable::Reference>>& dual_constraint_expressions,
                                                        const std::string& name_addendum) {
    auto const dus = soc_model().add_variables(normed_vector.normed_vector().size(), name_addendum + "_DSVar",
                                               NO_VARIABLE_LB, NO_VARIABLE_UB);
    for (size_t i = 0; i < normed_vector.normed_vector().size(); ++i) {
        auto const& affine = normed_vector.normed_vector().at(i);
        auto const du = dus.at(i);
        dual_objective += affine.constant() * du;
        for (auto const& svar: affine.linear().scaled_variables()) {
            dual_constraint_expressions[svar.variable().raw_id()] += svar.scale() * du;
        }
    }
    soc_model().add_constraint(SOCExpression<SOCVariable>::norm(dus, dual_norm_type(normed_vector.norm_type())) <= dv,
                               name_addendum + "_Dual_SOC");
}

void
AffineAdjustablePolicySolver::add_dual_of_uvar_bounds(AffineExpression<SOCVariable::Reference>& dual_objective,
                                                      std::vector<AffineExpression<SOCVariable::Reference>>& dual_constraint_expressions,
                                                      std::string const& name_addendum) {
    for (auto const& var: model().uncertainty_variables()) {
        auto& lhs = dual_constraint_expressions[var.id().raw_id()];
        if (var.lb() != NO_VARIABLE_LB) {
            auto const dv = soc_model().add_variable(name_addendum + "_LBD_" + var.name(), NO_VARIABLE_LB, 0);
            dual_objective += -var.lb() * dv;
            lhs += 1 * dv;
        }
        if (var.ub() != NO_VARIABLE_UB) {
            auto const dv = soc_model().add_variable(name_addendum + "_UBD_" + var.name(), 0, NO_VARIABLE_UB);
            dual_objective += -var.ub() * dv;
            lhs += 1 * dv;
        }
    }
}

void
AffineAdjustablePolicySolver::add_dual_of_expression(AffineExpression<SOCVariable::Reference>& dual_objective,
                                                     std::vector<AffineExpression<SOCVariable::Reference>>& dual_constraint_expressions,
                                                     RoAffineExpression const& expr) {
    for (auto const& suvar: expr.uncertainties().scaled_variables()) {
        dual_constraint_expressions[suvar.variable().raw_id()] += suvar.scale();
    }
    for (auto const& sdvar: expr.decisions().scaled_variables()) {
        for (auto const& dependency: sdvar.variable()->dependencies()) {
            dual_constraint_expressions[dependency.uncertainty_variable().raw_id()] +=
                    sdvar.scale() * adjustable_factor(dependency);
        }
        dual_objective += sdvar.scale() * adjustable_constant(sdvar.variable());
    }
    for (auto const& usdvar: expr.uncertainty_decisions().scaled_variables()) {
        helpers::exception_check(usdvar.variable().decision_variable()->dependencies().empty(),
                                 "Uncertainty Scaled Variables are not supported for recourse decisions");
        dual_constraint_expressions[usdvar.variable().decision_variable().raw_id()] +=
                usdvar.scale() * adjustable_constant(usdvar.variable().decision_variable());
    }
    dual_objective += expr.constant();
}

void AffineAdjustablePolicySolver::build_implementation() {
    build_variables();
    build_objective();
    build_constraints();
    _soc_solver = std::make_unique<solvers::GurobiSOCSolver>(soc_model());
}

void AffineAdjustablePolicySolver::solve_implementation() {
    set_parameters_to_other(soc_solver());
    soc_solver().solve();
    set_results_from_other(soc_solver());
}

AffineSolution AffineAdjustablePolicySolver::affine_solution(DecisionVariable::Index const dvar) const {
    std::map<UncertaintyVariable::Index, double> dependent_scales;
    for (auto const& dependency: dvar->dependencies()) {
        dependent_scales[dependency.uncertainty_variable()] = adjustable_factor(dependency)->solution();
    }
    return AffineSolution(adjustable_constant(dvar)->solution(), dependent_scales);
}

void AffineAdjustablePolicySolver::build_variables() {
    for (auto const& var: model().decision_variables()) {
        _adjustable_factors.emplace_back();
        for (auto const& dependency: var.dependencies()) {
            _adjustable_factors.back().emplace_back(
                    soc_model().add_variable("AV_" + var.name() + dependency.uncertainty_variable()->name(),
                                             NO_VARIABLE_LB, NO_VARIABLE_UB));
        }
    }
    for (auto const& var: model().decision_variables()) {
        _adjustable_constants.emplace_back(soc_model().add_variable("B_" + var.name(), NO_VARIABLE_LB, NO_VARIABLE_UB));
    }
}

void AffineAdjustablePolicySolver::build_constraints() {
    for (auto const& constr: model().constraints()) {
        add_ro_constraint(constr);
    }
    for (auto const& var: model().decision_variables()) {
        if (var.lb() != NO_VARIABLE_LB) {
            add_ro_constraint({var.reference() >= var.lb(), "LB_" + var.name()});
        }
        if (var.ub() != NO_VARIABLE_UB) {
            add_ro_constraint({var.reference() <= var.ub(), "UB_" + var.name()});
        }
    }
}

void AffineAdjustablePolicySolver::build_objective() {
    switch (model().objective().sense()) {
        case ObjectiveSense::MAX: {
            auto const obj = add_counterpart_constraints_for_minimization(
                    RoAffineExpression(model().objective().expression()),
                    "Objective");
            soc_model().add_objective({ObjectiveSense::MAX, obj});
            return;
        }
        case ObjectiveSense::MIN: {
            auto const obj = add_counterpart_constraints_for_minimization(
                    -RoAffineExpression(model().objective().expression()),
                    "Objective");
            soc_model().add_objective({ObjectiveSense::MIN, -obj});
            return;
        }
    }
}

SOCVariable::Reference const& AffineAdjustablePolicySolver::adjustable_factor(
        DecisionVariable::Dependency const& dependency) const {
    return _adjustable_factors.at(dependency.decision_variable().raw_id()).at(dependency.id().raw_id());
}

SOCVariable::Reference const& AffineAdjustablePolicySolver::adjustable_constant(DecisionVariable::Index id) const {
    return _adjustable_constants.at(id.raw_id());
}

SolutionRealization
AffineAdjustablePolicySolver::specific_solution(std::vector<double> const& uncertainty_realization) const {
    std::vector<double> solutions(model().num_dvars(), 0);
    for (auto const& dvar: model().decision_variables()) {
        solutions.at(dvar.id().raw_id()) = adjustable_constant(dvar.id())->solution();
        for (auto const& dependency: dvar.dependencies()) {
            solutions.at(dvar.id().raw_id()) +=
                    uncertainty_realization.at(dependency.uncertainty_variable().raw_id())
                    * adjustable_factor(dependency)->solution();
        }
    }
    return SolutionRealization(model(), uncertainty_realization, solutions);
}

void AffineAdjustablePolicySolver::add_average_reoptimization() {
    add_reoptimization_objective_for_realization(
            UncertaintyRealization(
                    std::vector<double>(model().num_uvars(), 1.)
            )
    );
}

void
AffineAdjustablePolicySolver::add_reoptimization_objective_for_realization(UncertaintyRealization const& realization) {
    helpers::exception_check(built(), "Can only add extra objectives, when base model is built!");
    std::vector<AffineExpression<SOCVariable::Reference>> decision_substitutions;
    for (auto const& dvar: model().decision_variables()) {
        AffineExpression<SOCVariable::Reference> replacement(adjustable_constant(dvar.id()));
        for (auto const& dependency: dvar.dependencies()) {
            replacement += adjustable_factor(dependency);
        }
        decision_substitutions.emplace_back(replacement);
    }

    soc_model().add_objective({SOCModel::Objective(
            model().objective().sense(),
            model().objective().expression().substitute_to_other_affine<SOCVariable::Reference>(
                    decision_substitutions,
                    realization.values()
            ))});
}

SOCModel& AffineAdjustablePolicySolver::soc_model() {
    return _soc_model;
}

solvers::SOCSolverBase& AffineAdjustablePolicySolver::soc_solver() {
    return *_soc_solver;
}


AffinePolicyExpectationHelper::AffinePolicyExpectationHelper(ROModel const& model) : _model(model) {}

std::vector<std::vector<double>> AffinePolicyExpectationHelper::expected_adjustable_factor_scales(
        RoAffineExpression const& expression) const {
    return model().expectation_provider().expected_value(
            [&](UncertaintyRealization const& realization) {
                std::vector<std::vector<double>> factors(model().num_dvars());
                for (auto const& dvar: model().decision_variables()) {
                    factors[dvar.id().raw_id()] = std::vector<double>(dvar.num_dependencies(), 0.);
                }
                for (auto const& svar: expression.decisions().scaled_variables()) {
                    for (auto const& dependency: svar.variable()->dependencies()) {
                        factors.at(svar.variable().raw_id()).at(dependency.id().raw_id()) +=
                                svar.scale() * realization.value(dependency.uncertainty_variable());
                    }
                }
                for (auto const& svar: expression.uncertainty_decisions().scaled_variables()) {
                    for (auto const& dependency: svar.variable().decision_variable()->dependencies()) {
                        factors.at(svar.variable().decision_variable().raw_id()).at(dependency.id().raw_id()) +=
                                svar.scale() *
                                realization.value(dependency.uncertainty_variable()) *
                                realization.value(svar.variable().uncertainty_variable());
                    }
                }
                return factors;
            });
}

std::vector<double> AffinePolicyExpectationHelper::expected_adjustable_constant_scales(
        RoAffineExpression const& expression) const {
    return model().expectation_provider().expected_value(
            [&](UncertaintyRealization const& realization) {
                std::vector<double> factors(model().num_dvars(), 0.);
                for (auto const& svar: expression.decisions().scaled_variables()) {
                    factors.at(svar.variable().raw_id()) +=
                            svar.scale();
                }
                for (auto const& svar: expression.uncertainty_decisions().scaled_variables()) {
                    factors.at(svar.variable().decision_variable().raw_id()) +=
                            svar.scale() *
                            realization.value(svar.variable().uncertainty_variable());
                }
                return factors;
            });
}

double AffinePolicyExpectationHelper::expected_constant(
        RoAffineExpression const& expression) const {
    return model().expectation_provider().expected_value(
            [&](UncertaintyRealization const& realization) {
                return expression.uncertainties().value(realization);
            })
           + expression.constant();
}

ROModel const& AffinePolicyExpectationHelper::model() const {
    return _model;
}


}