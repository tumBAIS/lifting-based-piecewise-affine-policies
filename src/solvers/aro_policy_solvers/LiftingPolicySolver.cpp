#include "LiftingPolicySolver.h"

#include <utility>

namespace robust_model {

SingleDirectionBreakPoints::SingleDirectionBreakPoints(helpers::SmartIndex<SingleDirectionBreakPoints> const& id,
                                                       SingleDirectionBreakPoints::BreakPointsSeries break_points,
                                                       SingleDirectionBreakPoints::BreakPointDirection break_point_direction)
        : IndexedObject<SingleDirectionBreakPoints>(id),
          _break_points(std::move(break_points)),
          _break_point_direction(std::move(break_point_direction)) {
    helpers::exception_check(break_points_disjoint(), "Break points are not disjoint!");
}

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
           break_point(i - 1) :
           break_point_direction().lb();
}

SingleDirectionBreakPoints::BreakPoint SingleDirectionBreakPoints::break_point(size_t i) const {
    return (i < break_points().size())
           ? break_points().at(i) :
           break_point_direction().ub();
}

bool SingleDirectionBreakPoints::break_points_disjoint() const {
    for (size_t i = 0; i <= break_points().size(); ++i) {
        if (previous_break_point(i) >= break_point(i))
            return false;
    }
    return true;
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
    auto const id = base_add_object(cleaned_break_points(break_points, break_point_direction), break_point_direction);
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

void LiftingPolicySolver::add_eta_induced_breakpoints(size_t num_pieces) {
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

void LiftingPolicySolver::add_full_eta_induced_breakpoints() {
    add_eta_induced_breakpoints(model().num_uvars());
}

void LiftingPolicySolver::build_implementation() {
    helpers::exception_check(not built(), "Only build model once!");
    if (_all_simple_axis_aligned) {
        build_axis_aligned_model();
    } else {
        helpers::exception_throw("Not implemented yet!");
    }
    if (model().has_expectation_provider()) {
        lifted_model().set_expectation_provider(std::make_unique<SOExpectationProviderLifted>(
                model().expectation_provider(), *this));
    }
    if (_use_dual_solver) {
        _affine_dual_model = std::make_unique<AffineAdjustablePolicyDualSolver>(_lifted_model);
    } else {
        _affine_primal_model = std::make_unique<AffineAdjustablePolicySolver>(_lifted_model);
    }
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
    //TODO CHECK THAT ALL DEPENDENCIES ARE INDUCED BY PERIOD, OTHERWISE WE DON'T HAVE THE EXPECTED BEHAVIOUR!
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

LiftingPolicySolver::BreakPointsSeries
LiftingPolicySolver::cleaned_break_points(LiftingPolicySolver::BreakPointsSeries const& break_points,
                                          LiftingPolicySolver::BreakPointDirection const& break_point_direction) const {
    BreakPointsSeries cleaned_break_points;
    for (auto breakpoint: break_points) {
        if ((cleaned_break_points.empty() or
             (breakpoint > cleaned_break_points.back() + breakpoint_distance_threshold))
            and
            ((breakpoint > break_point_direction.lb() + breakpoint_distance_threshold) and
             (breakpoint < break_point_direction.ub() - breakpoint_distance_threshold))
                ) {
            cleaned_break_points.emplace_back(breakpoint);
        }
    }
    helpers::warning_check(cleaned_break_points.size() == break_points.size(),
                           "Break points were not strictly increasing!");
    return cleaned_break_points;
}

void LiftingPolicySolver::solve_implementation() {
    helpers::exception_check(built(), "Can only solve built model!");
    set_parameters_to_other(affine_model());
    affine_model().solve();
    set_results_from_other(affine_model());

    if (_use_dual_solver) {
        size_t non_improvement_counter = 0;
        double runtime = affine_model().runtime();
        while ((affine_model().status() == solvers::SolverBase::Status::OPTIMAL)
               and
               add_most_violated_cut()
               and
               (non_improvement_counter < 20)
                ) {
            double previous_objective = affine_model().objective_value();
            affine_model().set_runtime_limit(runtime_limit() - runtime);
            affine_model().solve();
            set_results_from_other(affine_model());
            double improvement = std::abs(previous_objective - affine_model().objective_value());
            runtime += affine_model().runtime();
            ++non_improvement_counter;
            if (improvement > improvement_threshold) {
                non_improvement_counter = 0;
            }
        }
        set_runtime(runtime);
    }
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

bool LiftingPolicySolver::add_most_violated_cut() {
    bool cut_added = false;
    for (size_t constr_id = 0; constr_id < lifted_model().constraints().size(); ++constr_id) {
        if (lifted_model().constraints().at(constr_id).sense() == ConstraintSense::EQ)
            continue;
        for (auto const& constraint_set: lifted_model().uncertainty_set().constraint_sets()) {
            cut_added = cut_added or add_most_violated_cut(constr_id, constraint_set);
        }
    }
    return cut_added;
}

bool LiftingPolicySolver::add_most_violated_cut(
        size_t constr_id,
        UncertaintySetConstraintsSet::Index const& constraint_set
) {
    AffineAdjustablePolicyDualSolver::ScaledUncertainties const& lifted_dual_uncertainty_variables =
            affine_dual_model().scaled_uncertainty_variables(constr_id, constraint_set);
    if (lifted_dual_uncertainty_variables.scale->solution() == 0.)
        return false;
    auto const symmetric_cut_encoding = find_most_violated_cut(lifted_dual_uncertainty_variables);
    if (symmetric_cut_encoding.critical_layer == num_positive_quadrant_pieces() - 1 and
        symmetric_cut_encoding.active_indices.empty())
        return false;

    AffineExpression<UncertaintyVariable::Reference> lhs;
    for (size_t j = num_positive_quadrant_pieces() - 1; j > symmetric_cut_encoding.critical_layer; --j) {
        for (auto const& uvar: model().uncertainty_variables()) {
            lhs += positive_quadrant_uncertainty_expressions(j, uvar.id());
        }
    }
    for (auto const& id: symmetric_cut_encoding.active_indices) {
        lhs += positive_quadrant_uncertainty_expressions(
                symmetric_cut_encoding.critical_layer, model().uncertainty_variables().at(id).id());
    }
    affine_dual_model().add_uncertainty_constraint(
            {lhs <= max_one_norm_k_active(symmetric_cut_encoding.active_indices.size()), "cut"},
            constraint_set, constr_id);
    return true;
}

size_t LiftingPolicySolver::num_positive_quadrant_pieces() const {
    if (all_symmetric_axis_aligned_breakpoints() and model().uncertainty_set().symmetric()) {
        return _lifted_uncertainty_variables.begin()->size() / 2;
    }
    if (model().uncertainty_set().non_negative()) {
        return _lifted_uncertainty_variables.begin()->size();
    }
    helpers::exception_throw("Illegal Case!");
    return 0;
}

AffineExpression<UncertaintyVariable::Reference>
LiftingPolicySolver::positive_quadrant_uncertainty_expressions(
        size_t positive_quadrant_piece,
        UncertaintyVariable::Index const& uvar) const {
    if (all_symmetric_axis_aligned_breakpoints() and model().uncertainty_set().symmetric()) {
        auto const& lifted_uvars = _lifted_uncertainty_variables.at(uvar.raw_id());
        return lifted_uvars.at(lifted_uvars.size() / 2 - positive_quadrant_piece - 1).ub() -
               lifted_uvars.at(lifted_uvars.size() / 2 - positive_quadrant_piece - 1) +
               lifted_uvars.at(lifted_uvars.size() / 2 + positive_quadrant_piece);
    }
    if (model().uncertainty_set().non_negative()) {
        auto const& lifted_uvars = _lifted_uncertainty_variables.at(uvar.raw_id());
        return AffineExpression<UncertaintyVariable::Reference>{lifted_uvars.at(positive_quadrant_piece)};
    }
    helpers::exception_throw("Illegal Case!");
    return {};
}


double LiftingPolicySolver::positive_quadrant_break_point(size_t positive_quadrant_piece) const {
    if (all_symmetric_axis_aligned_breakpoints() and model().uncertainty_set().symmetric()) {
        return objects().begin()->previous_break_point(
                (objects().begin()->break_points().size() + 1) / 2 + positive_quadrant_piece);
    }
    if (model().uncertainty_set().non_negative()) {
        return objects().begin()->previous_break_point(positive_quadrant_piece);
    }
    helpers::exception_throw("Illegal Case!");
    return 0.;
}

double LiftingPolicySolver::max_one_norm_k_active(size_t k) const {
    return model().uncertainty_set().max_one_norm_k_active(k);
}

LiftingPolicySolver::CutEncoding LiftingPolicySolver::find_most_violated_cut(
        AffineAdjustablePolicyDualSolver::ScaledUncertainties const& scaled_uncertainties
) const {
    std::vector<std::vector<double>> deltas(num_positive_quadrant_pieces(), std::vector<double>(model().num_uvars()));
    for (size_t j = 0; j < num_positive_quadrant_pieces(); ++j) {
        for (auto const& uvar: model().uncertainty_variables()) {
            deltas[j][uvar.id().raw_id()] =
                    positive_quadrant_uncertainty_expressions(j, uvar.id()).value(scaled_uncertainties);
        }
    }

    double obj_min = cut_violation_threshold;
    size_t i_min = 0;
    size_t j_min = num_positive_quadrant_pieces() - 1;
    double obj_current = 0;

    for (size_t j = num_positive_quadrant_pieces(); j-- > 0;) {
        auto [indices, delta_j] = helpers::sort(deltas[j]);
        for (size_t i = 0; i < delta_j.size(); ++i) {
            obj_current += delta_j[i];
            if (max_one_norm_k_active(i + 1) - max_one_norm_k_active(i) >= positive_quadrant_break_point(j + 1))
                obj_current -= positive_quadrant_break_point(j + 1) - positive_quadrant_break_point(j);
            else if (max_one_norm_k_active(i + 1) - max_one_norm_k_active(i) >= positive_quadrant_break_point(j))
                obj_current -=
                        max_one_norm_k_active(i + 1) - max_one_norm_k_active(i) - positive_quadrant_break_point(j);
            if (obj_current > obj_min) {
                obj_min = obj_current;
                i_min = i + 1;
                j_min = j;
            }
        }
    }

    size_t critical_layer = j_min;

    auto [indices, delta_j] = helpers::sort(deltas[critical_layer]);
    std::vector<size_t> active_indices(i_min);
    for (size_t i = 0; i < i_min; ++i) {
        active_indices[i] = indices[i];
    }
    return {critical_layer, active_indices};
}

bool LiftingPolicySolver::all_rotational_invariant_axis_aligned_breakpoints() const {
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

bool LiftingPolicySolver::all_symmetric_axis_aligned_breakpoints() const {
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

solvers::AROPolicySolverBase& LiftingPolicySolver::affine_model() {
    if (_use_dual_solver) {
        return affine_dual_model();
    }
    return affine_primal_model();
}

solvers::AROPolicySolverBase const& LiftingPolicySolver::affine_model() const {
    if (_use_dual_solver) {
        return affine_dual_model();
    }
    return affine_primal_model();
}

AffineAdjustablePolicySolver& LiftingPolicySolver::affine_primal_model() {
    helpers::exception_check(_affine_primal_model != nullptr, "Affine Primal Model not set!");
    return *_affine_primal_model;
}

AffineAdjustablePolicySolver const& LiftingPolicySolver::affine_primal_model() const {
    helpers::exception_check(_affine_primal_model != nullptr, "Affine Primal Model not set!");
    return *_affine_primal_model;
}

AffineAdjustablePolicyDualSolver& LiftingPolicySolver::affine_dual_model() {
    helpers::exception_check(_affine_dual_model != nullptr, "Affine Dual Model not set!");
    return *_affine_dual_model;
}

AffineAdjustablePolicyDualSolver const& LiftingPolicySolver::affine_dual_model() const {
    helpers::exception_check(_affine_dual_model != nullptr, "Affine Dual Model not set!");
    return *_affine_dual_model;
}

void LiftingPolicySolver::set_use_dual_solver(bool use_dual_solver) {
    _use_dual_solver = use_dual_solver;
}

void LiftingPolicySolver::set_breakpoint_tightening(bool breakpoint_tightening) {
    _breakpoint_tightening = breakpoint_tightening;
}

}