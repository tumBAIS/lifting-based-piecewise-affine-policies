#include "LiftingPolicySolver.h"

#include <utility>

namespace robust_model {

SingleDirectionBreakPoints::SingleDirectionBreakPoints(helpers::SmartIndex<SingleDirectionBreakPoints> const& id,
                                                       SingleDirectionBreakPoints::BreakPointsSeries break_points,
                                                       SingleDirectionBreakPoints::BreakPointDirection break_point_direction)
        : IndexedObject<SingleDirectionBreakPoints>(id),
          _break_points(std::move(break_points)),
          _break_point_direction(std::move(break_point_direction)) {}

bool SingleDirectionBreakPoints::simple_axis_aligned() const {
    if (break_point_direction().scaled_variables().size() != 1) {
        return false;
    }
    auto const& svar = break_point_direction().scaled_variables().front();
    return (svar.scale() == 1) and (svar.variable()->id().raw_id() == id().raw_id());
}

SingleDirectionBreakPoints::BreakPointsSeries const& SingleDirectionBreakPoints::break_points() const {
    return _break_points;
}

SingleDirectionBreakPoints::BreakPointDirection const& SingleDirectionBreakPoints::break_point_direction() const {
    return _break_point_direction;
}

UncertaintyVariable::Index const& SingleDirectionBreakPoints::axis_direction() const {
    helpers::exception_check(simple_axis_aligned(), "Can only get axis direction of aligned direction!");
    return break_point_direction().scaled_variables().front().variable()->id();
}

std::vector<double>
SingleDirectionBreakPoints::lifted_uncertainty_realization(
        UncertaintyRealization const& uncertainty_realization) const {
    double const directed_realization = break_point_direction().value(uncertainty_realization);
    std::vector<double> lifted_realization(break_points().size() + 1);
    for (size_t i = 0; i <= break_points().size(); ++i) {
        double const lb = previous_break_point(i);
        double const ub = break_point(i);
        lifted_realization.at(i) = std::max(std::min(directed_realization, ub) - lb, 0.);
    }
    return lifted_realization;
}

SingleDirectionBreakPoints::BreakPoint SingleDirectionBreakPoints::previous_break_point(size_t i) const {
    return (i > 0) ?
           break_points().at(i - 1) :
           break_point_direction().lb();
}

SingleDirectionBreakPoints::BreakPoint SingleDirectionBreakPoints::break_point(size_t i) const {
    return (i < break_points().size())
           ? break_points().at(i) :
           break_point_direction().ub();
}

SOExpectationProviderLifted::SOExpectationProviderLifted(SOExpectationProvider const& base_expectation_provider,
                                                         LiftingPolicySolver const& lifting_policy_solver)
        : _base_expectation_provider(base_expectation_provider), _lifting_policy_solver(lifting_policy_solver) {}

double SOExpectationProviderLifted::expected_value(
        std::function<double(UncertaintyRealization const&)> const& fct) const {
    return _base_expectation_provider.expected_value(
            [&](UncertaintyRealization const& realization) {
                return fct(UncertaintyRealization(
                        _lifting_policy_solver.lifted_uncertainty_realization(realization.values())));
            }
    );
}

std::vector<double> SOExpectationProviderLifted::expected_value(
        std::function<std::vector<double>(UncertaintyRealization const&)> const& fct) const {
    return _base_expectation_provider.expected_value(
            [&](UncertaintyRealization const& realization) {
                return fct(UncertaintyRealization(
                        _lifting_policy_solver.lifted_uncertainty_realization(realization.values())));
            }
    );
}

std::vector<std::vector<double>> SOExpectationProviderLifted::expected_value(
        std::function<std::vector<std::vector<double>>(UncertaintyRealization const&)> const& fct) const {
    return _base_expectation_provider.expected_value(
            [&](UncertaintyRealization const& realization) {
                return fct(UncertaintyRealization(
                        _lifting_policy_solver.lifted_uncertainty_realization(realization.values())));
            }
    );
}

LiftingPolicySolver::LiftingPolicySolver(ROModel const& original_model) :
        solvers::AROPolicySolverBase(original_model) {}

void LiftingPolicySolver::add_break_points(LiftingPolicySolver::BreakPointsSeries const& break_points,
                                           LiftingPolicySolver::BreakPointDirection const& break_point_direction) {
//    helpers::exception_check(not _breakpoint_tightening,
//                             "If domination breakpoints tightening shall be used, no additional breakpoints may be set!");
//TODO Add an exception check for validity of breakpoints - i.e. strictly increasing!
    auto const id = base_add_object(break_points, break_point_direction);
    _all_simple_axis_aligned = _all_simple_axis_aligned and id->simple_axis_aligned();
}

void LiftingPolicySolver::add_equidistant_breakpoints(size_t num_pieces) {
    for (auto const& uvar: model().uncertainty_variables()) {
        BreakPointsSeries breakpoints;
        for (size_t breakpoint_id = 1; breakpoint_id < num_pieces; ++breakpoint_id) {
            breakpoints.emplace_back(
                    uvar.lb() + double(breakpoint_id) * (uvar.ub() - uvar.lb()) / double(num_pieces));
        }
        add_break_points(breakpoints,
                         BreakPointDirection{uvar.reference()});
    }
}

void LiftingPolicySolver::add_kappa_induced_breakpoints(size_t num_pieces) {
    helpers::exception_check(model().uncertainty_set().rotational_invariant(),
                             "Has to be rotational invariant!");
    auto const symmetric = model().uncertainty_set().symmetric();
    auto const non_negative = model().uncertainty_set().non_negative();
    helpers::exception_check(symmetric or non_negative,
                             "Has to be symmetric or non negative!");
    auto const k_active_calc = [&](size_t const breakpoint_id) {
        return (breakpoint_id * model().uncertainty_set().num_variables()) / num_pieces;
    };
    for (auto const& uvar: model().uncertainty_variables()) {
        BreakPointsSeries breakpoints;
        if (symmetric) {
            for (size_t breakpoint_id = 1; breakpoint_id < num_pieces; ++breakpoint_id) {
                size_t const k_active = k_active_calc(breakpoint_id);
                breakpoints.emplace_back(model().uncertainty_set().max_one_norm_k_active(k_active) -
                                         model().uncertainty_set().max_one_norm_k_active(k_active + 1));
            }
            breakpoints.emplace_back(0);
        }
        for (size_t breakpoint_id = num_pieces - 1; breakpoint_id > 0; --breakpoint_id) {
            size_t const k_active = k_active_calc(breakpoint_id);
            breakpoints.emplace_back(model().uncertainty_set().max_one_norm_k_active(k_active + 1) -
                                     model().uncertainty_set().max_one_norm_k_active(k_active));
        }
        add_break_points(breakpoints,
                         BreakPointDirection{uvar.reference()});
    }
}

void LiftingPolicySolver::add_full_kappa_induced_breakpoints() {
    add_kappa_induced_breakpoints(model().num_uvars());
}

void LiftingPolicySolver::build_implementation() {
    helpers::exception_check(not built(), "Only build model once!");
    if (_all_simple_axis_aligned) {
        build_axis_aligned_model();
    } else {
        helpers::exception_throw("Not implemented yet!");
    }
//    if (_breakpoint_tightening) {
//        add_domination_motivated_tightening_constraint();
//    }
    if (model().has_expectation_provider()) {
        lifted_model().set_expectation_provider(std::make_unique<SOExpectationProviderLifted>(
                model().expectation_provider(), *this));
    }
    _affine_model = std::make_unique<AffineAdjustablePolicySolver>(_lifted_model);
    affine_model().build();
}

void LiftingPolicySolver::build_axis_aligned_model() {
    helpers::exception_check(_all_simple_axis_aligned, "Not all break points are simple axis aligned!");

    axis_aligned_lifted_model_add_lifted_uncertainty_variables();
    axis_aligned_lifted_model_add_retracted_uncertainty_constraints();

    add_decision_variables();
    axis_aligned_lifted_model_add_retracted_constraints();
    axis_aligned_lifted_model_add_retracted_objective();

    add_box_tightening_constraints();
    if (all_rotational_invariant_axis_aligned_breakpoints() and
        model().uncertainty_set().rotational_invariant() and
        _breakpoint_tightening) {
        add_rotational_invariant_tightening_constraints();
    }
}


void LiftingPolicySolver::add_decision_variables() {
    for (auto const& dvar: model().decision_variables()) {
        _lifted_decision_variables.emplace_back(
                lifted_model().add_decision_variable(dvar.name(),
                                                     (dvar.has_period()) ? dvar.period() : std::optional<period_id>{},
                                                     dvar.lb(), dvar.ub()));
    }
}

void LiftingPolicySolver::axis_aligned_lifted_model_add_lifted_uncertainty_variables() {
    for (auto const& break_point_series: objects()) {
        _lifted_uncertainty_variables.emplace_back();
        auto const& udirection = break_point_series.axis_direction();
        for (size_t i = 0; i <= break_point_series.break_points().size(); ++i) {
            BreakPoint break_point = break_point_series.break_point(i);
            BreakPoint prev_break_point = break_point_series.previous_break_point(i);
            _lifted_uncertainty_variables.back().emplace_back(
                    lifted_model().add_uncertainty_variable(
                            udirection->name() + "_L" + std::to_string(i),
                            udirection->has_period() ? udirection->period() : std::optional<period_id>{},
                            0,
                            break_point - prev_break_point
                    )
            );
            if (i > 0) {
                lifted_model().add_uncertainty_constraint(
                        _lifted_uncertainty_variables.back().at(i) /
                        _lifted_uncertainty_variables.back().at(i).ub()
                        <=
                        _lifted_uncertainty_variables.back().at(i - 1) /
                        _lifted_uncertainty_variables.back().at(i - 1).ub(),
                        "BoundLiftedWithPrevious" + udirection->name() + "_" + std::to_string(i)
                );
            }
        }
        _lifted_uncertainty_retractions.emplace_back(
                udirection->lb(),
                LinearExpression<UncertaintyVariable::Reference>::sum(_lifted_uncertainty_variables.back())
        );
    }
}

void LiftingPolicySolver::axis_aligned_lifted_model_add_retracted_uncertainty_constraints() {
    for (auto const union_uncertainty_set: model().uncertainty_set().constraint_sets()) {
        if (union_uncertainty_set.raw_id() > 0) {
            lifted_model().add_uncertainty_constraint_set();
        }
        auto const lifted_union_uncertainty_set = lifted_model().uncertainty_set().constraint_sets().back();
        for (auto const& uconstr: model().uncertainty_set().uncertainty_constraints(union_uncertainty_set)) {
            lifted_model().add_uncertainty_constraint(
                    uconstr.substitute<UncertaintyVariable>(
                            _lifted_uncertainty_retractions
                    ),
                    lifted_union_uncertainty_set);
        }
    }
}

void LiftingPolicySolver::axis_aligned_lifted_model_add_retracted_constraints() {
    for (auto const& roconstr: model().constraints()) {
        lifted_model().add_constraint(
                {
                        roconstr.name(),
                        roconstr.sense(),
                        roconstr.expression().substitute(_lifted_decision_variables,
                                                         _lifted_uncertainty_retractions)
                });
    }
}

void LiftingPolicySolver::axis_aligned_lifted_model_add_retracted_objective() {
    auto const& old_obj = model().objective();
    lifted_model().set_objective(
            old_obj.expression().substitute(_lifted_decision_variables, _lifted_uncertainty_retractions),
            old_obj.sense());
}

void LiftingPolicySolver::add_box_tightening_constraints() {
    helpers::exception_check(_all_simple_axis_aligned,
                             "Boxes only stay boxes, when everything is nicely axis aligned");
    auto const lbs = model().uncertainty_set().lower_bounds();
    auto const ubs = model().uncertainty_set().upper_bounds();
    for (auto const& constraint_set: model().uncertainty_set().constraint_sets()) {
        if (not constraint_set->is_box())
            continue;
        auto const lifted_constraint_set = lifted_model().uncertainty_set().constraint_sets().at(
                constraint_set.raw_id());

        auto const [box_lbs, box_ubs] = get_box_bounds(lbs, ubs, constraint_set);

        for (auto const& break_point_series: objects()) {
            auto const direction_var = break_point_series.axis_direction();
            auto const box_lb = box_lbs.at(direction_var.raw_id());
            auto const box_ub = box_ubs.at(direction_var.raw_id());
            add_box_tightening_constraint_in_direction(box_lb, box_ub, break_point_series, lifted_constraint_set);
        }
    }
}

void LiftingPolicySolver::add_rotational_invariant_tightening_constraints() {
    helpers::exception_check(all_rotational_invariant_axis_aligned_breakpoints() and
                             model().uncertainty_set().rotational_invariant(),
                             "Only add rotational invariant tightening for rotational invariant sets!");
    size_t const lift_vars_per_variable = _lifted_uncertainty_variables.front().size();

    if (all_symmetric_axis_aligned_breakpoints() and model().uncertainty_set().symmetric()) {
        AffineExpression<UncertaintyVariable::Reference> lhs;
        for (size_t i = 0; i < lift_vars_per_variable / 2; ++i) {
            double break_point = -objects().begin()->break_point(i);
            for (auto const& lifted_vars: _lifted_uncertainty_variables) {
                lhs += lifted_vars.at(i).ub() - lifted_vars.at(i);
                lhs += lifted_vars.at(lift_vars_per_variable - i - 1);
            }
            lifted_model().add_uncertainty_constraint(
                    lhs <= max_rotational_invariant_outside_budget(break_point),
                    "RotationalBound_BP" + std::to_string(i)
            );
        }
    }
    if (model().uncertainty_set().non_negative()) {
        LinearExpression<UncertaintyVariable::Reference> lhs;
        for (int i = int(lift_vars_per_variable) - 1; i >= 0; --i) {
            double break_point = objects().begin()->previous_break_point(i);
            for (auto const& lifted_vars: _lifted_uncertainty_variables) {
                lhs += lifted_vars.at(i);
            }
            lifted_model().add_uncertainty_constraint(
                    lhs <= max_rotational_invariant_outside_budget(break_point),
                    "RotationalBound_BP" + std::to_string(i)
            );
        }
    }
}

double LiftingPolicySolver::max_rotational_invariant_outside_budget(double bound) const {
    double max_outside_budget = 0;
    for (size_t k = 1; k <= model().num_uvars(); ++k) {
        max_outside_budget = std::max(max_outside_budget,
                                      model().uncertainty_set().max_one_norm_k_active(k) - double(k) * bound);
    }
    return max_outside_budget;
}

std::pair<std::vector<double>, std::vector<double>> LiftingPolicySolver::get_box_bounds(
        std::vector<double> lbs,
        std::vector<double> ubs,
        UncertaintySetConstraintsSet::Index const& box_constraint_set) const {
    for (auto const& constr: box_constraint_set->constraints()) {
        auto const var = constr.soc_expression().affine().linear().scaled_variables().front().variable();
        auto const rhs_bound = -constr.soc_expression().affine().constant();
        switch (constr.sense()) {
            case ConstraintSense::LEQ:
                ubs.at(var.raw_id()) = std::min(ubs.at(var.raw_id()), rhs_bound);
                break;
            case ConstraintSense::GEQ:
                lbs.at(var.raw_id()) = std::max(lbs.at(var.raw_id()), rhs_bound);
                break;
            case ConstraintSense::EQ:
                ubs.at(var.raw_id()) = std::min(ubs.at(var.raw_id()), rhs_bound);
                lbs.at(var.raw_id()) = std::max(lbs.at(var.raw_id()), rhs_bound);
                break;
        }
    }
    return {std::move(lbs), std::move(ubs)};
}

void LiftingPolicySolver::add_box_tightening_constraint_in_direction(
        double box_lb, double box_ub,
        SingleDirectionBreakPoints const& break_point_series,
        UncertaintySetConstraintsSet::Index const& lifted_constraint_set) {

    if (_use_old_box_constraints) {
        add_box_tightening_constraint_in_direction_old(box_lb, box_ub, break_point_series, lifted_constraint_set);
        return;
    }

    auto const& udirection = break_point_series.axis_direction();

    for (size_t i = 0; i <= break_point_series.break_points().size(); ++i) {
        BreakPoint break_point = break_point_series.break_point(i);
        BreakPoint prev_break_point = break_point_series.previous_break_point(i);

        double const this_lower_lift = std::min(std::max(box_lb - prev_break_point, 0.),
                                                break_point - prev_break_point);
        double const this_upper_lift = std::min(std::max(box_ub - prev_break_point, 0.),
                                                break_point - prev_break_point);

        lifted_model().add_uncertainty_constraint(
                _lifted_uncertainty_variables.at(break_point_series.id().raw_id()).at(i)
                >= this_lower_lift,
                "BoundLiftedBoxLB" + std::to_string(lifted_constraint_set.raw_id()) + "_" + udirection->name() + "_" +
                std::to_string(i),
                lifted_constraint_set
        );
        lifted_model().add_uncertainty_constraint(
                _lifted_uncertainty_variables.at(break_point_series.id().raw_id()).at(i)
                <= this_upper_lift,
                "BoundLiftedBoxUB" + std::to_string(lifted_constraint_set.raw_id()) + "_" + udirection->name() + "_" +
                std::to_string(i),
                lifted_constraint_set
        );

        if (box_lb < break_point and break_point < box_ub) {
            BreakPoint next_break_point = break_point_series.break_point(i + 1);

            double const next_upper_lift = std::min(box_ub - break_point, next_break_point - break_point);

            lifted_model().add_uncertainty_constraint(
                    (_lifted_uncertainty_variables.at(break_point_series.id().raw_id()).at(i) - this_lower_lift) /
                    (_lifted_uncertainty_variables.at(break_point_series.id().raw_id()).at(i).ub() - this_lower_lift)
                    >=
                    _lifted_uncertainty_variables.at(break_point_series.id().raw_id()).at(i + 1) / next_upper_lift,
                    "BoundLiftedWithPreviousBox" + std::to_string(lifted_constraint_set.raw_id()) + "_" +
                    udirection->name() + "_" + std::to_string(i),
                    lifted_constraint_set
            );
        }
    }


}

void LiftingPolicySolver::add_box_tightening_constraint_in_direction_old(
        double box_lb, double box_ub,
        SingleDirectionBreakPoints const& break_point_series,
        UncertaintySetConstraintsSet::Index const& lifted_constraint_set) {

    auto const direction_var = break_point_series.axis_direction();

    size_t just_inside_lb_index = 0;
    size_t one_outside_ub_index = break_point_series.break_points().size() + 1;
    SOCExpression<UncertaintyVariable> lhs;
    for (int break_point_index = 0;
         break_point_index <= break_point_series.break_points().size(); ++break_point_index) {
        auto const interval_lb = break_point_series.previous_break_point(break_point_index);
        auto const interval_ub = break_point_series.break_point(break_point_index);
        if (interval_ub <= box_lb) {
            ++just_inside_lb_index;
            lhs -= _lifted_uncertainty_variables.at(direction_var.raw_id()).at(break_point_index);
            lhs += interval_ub - interval_lb;
        }
        if (interval_lb >= box_ub) {
            --one_outside_ub_index;
            lhs += _lifted_uncertainty_variables.at(direction_var.raw_id()).at(break_point_index);
        }
    }
    lifted_model().add_uncertainty_constraint(
            lhs <= 0,
            "LiftedBoxConstrOuter_" + direction_var->name() + "_US" +
            std::to_string(lifted_constraint_set.raw_id()),
            lifted_constraint_set);

    if (just_inside_lb_index == one_outside_ub_index)
        return;

    double lb_overreach = break_point_series.break_point(just_inside_lb_index) - box_lb;
    double ub_overreach = box_ub - break_point_series.previous_break_point(one_outside_ub_index - 1);
    auto const lb_boundary_var = _lifted_uncertainty_variables.at(direction_var.raw_id()).at(
            just_inside_lb_index);
    auto const ub_boundary_var = _lifted_uncertainty_variables.at(direction_var.raw_id()).at(
            one_outside_ub_index - 1);
    if (lb_overreach < ub_overreach) {
        lifted_model().add_uncertainty_constraint(
                lhs + lb_boundary_var->ub() - lb_boundary_var <= lb_overreach,
                "LiftedBoxConstrOuterSmaller_" + direction_var->name() + "_US" +
                std::to_string(lifted_constraint_set.raw_id()),
                lifted_constraint_set);
    }
    if (ub_overreach < lb_overreach or (lb_boundary_var.raw_id() == ub_boundary_var.raw_id())) {
        lifted_model().add_uncertainty_constraint(
                lhs + ub_boundary_var <= ub_overreach,
                "LiftedBoxConstrOuterSmaller_" + direction_var->name() + "_US" +
                std::to_string(lifted_constraint_set.raw_id()),
                lifted_constraint_set);
    }
    if (lb_boundary_var.raw_id() != ub_boundary_var.raw_id()) {
        //TODO this can be made slightly tighter if we have smaller break point intervals on one side
        lifted_model().add_uncertainty_constraint(
                lhs + lb_boundary_var->ub() - lb_boundary_var + ub_boundary_var <=
                std::max(lb_overreach, ub_overreach),
                "LiftedBoxConstrOuterFull_" + direction_var->name() + "_US" +
                std::to_string(lifted_constraint_set.raw_id()),
                lifted_constraint_set);
    }
}

void LiftingPolicySolver::solve_implementation() {
    helpers::exception_check(built(), "Can only solve built model!");
    set_parameters_to_other(affine_model());
    affine_model().solve();
    set_results_from_other(affine_model());
}

SolutionRealization LiftingPolicySolver::specific_solution(std::vector<double> const& uncertainty_realization) const {
    return {model(),
            uncertainty_realization,
            affine_model().specific_solution(lifted_uncertainty_realization(uncertainty_realization)).solutions()};
}

std::vector<double>
LiftingPolicySolver::lifted_uncertainty_realization(std::vector<double> const& uncertainty_realization) const {
    UncertaintyRealization const realization(uncertainty_realization);
    std::vector<double> lifted_uncertainty(lifted_model().num_uvars());
    for (auto const& break_point_series: objects()) {
        auto const break_direction_lifting = break_point_series.lifted_uncertainty_realization(realization);
        for (size_t i = 0; i <= break_point_series.break_points().size(); ++i) {
            auto const uvar = _lifted_uncertainty_variables.at(break_point_series.id().raw_id()).at(i);
            lifted_uncertainty.at(uvar.raw_id()) = break_direction_lifting.at(i);
        }
    }
    return lifted_uncertainty;
}

bool LiftingPolicySolver::all_rotational_invariant_axis_aligned_breakpoints() {
    if (not _all_simple_axis_aligned) {
        return false;
    }
    if (num_objects() == 0) {
        return true;
    }
    auto const& series = objects().front().break_points();
    for (auto const& break_points: objects()) {
        if (break_points.break_points().size() != series.size()) {
            return false;
        }
        for (size_t i = 0; i < series.size(); ++i) {
            if (series.at(i) != break_points.break_point(i)) {
                return false;
            }
        }
    }
    return true;
}

bool LiftingPolicySolver::all_symmetric_axis_aligned_breakpoints() {
    if (not _all_simple_axis_aligned) {
        return false;
    }
    if (num_objects() == 0) {
        return true;
    }
    for (auto const& break_points: objects()) {
        for (size_t i = 0; i < break_points.break_points().size(); ++i) {
            if (break_points.break_point(break_points.break_points().size() - i - 1) != -break_points.break_point(i)) {
                return false;
            }
        }
    }
    return true;
}

ROModel& LiftingPolicySolver::lifted_model() {
    return _lifted_model;
}

ROModel const& LiftingPolicySolver::lifted_model() const {
    return _lifted_model;
}

AffineAdjustablePolicySolver& LiftingPolicySolver::affine_model() {
    return *_affine_model;
}

AffineAdjustablePolicySolver const& LiftingPolicySolver::affine_model() const {
    return *_affine_model;
}

void LiftingPolicySolver::set_use_old_box_constraints(bool use_old_box_constraints) {
    _use_old_box_constraints = use_old_box_constraints;
}

void LiftingPolicySolver::set_breakpoint_tightening(bool breakpoint_tightening) {
    _breakpoint_tightening = breakpoint_tightening;
}

}