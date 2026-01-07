#ifndef ROBUSTOPTIMIZATION_LIFTINGPOLICYSOLVER_H
#define ROBUSTOPTIMIZATION_LIFTINGPOLICYSOLVER_H

#include "AffineAdjustablePolicySolver.h"
#include "AffineAdjustablePolicyDualSolver.h"

namespace robust_model {

class LiftingPolicySolver;

class SingleDirectionBreakPoints : public helpers::IndexedObject<SingleDirectionBreakPoints> {
public:
    using BreakPoint = double;
    using BreakPointsSeries = std::vector<BreakPoint>;
    using BreakPointDirection = LinearExpression<UncertaintyVariable::Reference>;

    SingleDirectionBreakPoints(helpers::SmartIndex<SingleDirectionBreakPoints> const& id,
                               BreakPointsSeries break_points, BreakPointDirection break_point_direction);

    bool simple_axis_aligned() const;

    BreakPointsSeries const& break_points() const;

    BreakPointDirection const& break_point_direction() const;

    UncertaintyVariable::Index const& axis_direction() const;

    BreakPoint previous_break_point(size_t i) const;

    BreakPoint break_point(size_t i) const;

    std::vector<double> lifted_uncertainty_realization(UncertaintyRealization const& uncertainty_realization) const;

    bool break_points_disjoint() const;

private:
    BreakPointsSeries const _break_points;
    BreakPointDirection const _break_point_direction;
};

class SOExpectationProviderLifted : public SOExpectationProvider{
public:
    SOExpectationProviderLifted(SOExpectationProvider const& base_expectation_provider,
                                LiftingPolicySolver const& lifting_policy_solver);

    double expected_value(std::function<double(UncertaintyRealization const&)> const& fct) const final;
    std::vector<double> expected_value(std::function<std::vector<double>(UncertaintyRealization const&)> const& fct) const final;
    std::vector<std::vector<double>> expected_value(std::function<std::vector<std::vector<double>>(UncertaintyRealization const&)> const& fct) const override;

private:
    SOExpectationProvider const& _base_expectation_provider;
    LiftingPolicySolver const& _lifting_policy_solver;
};


class LiftingPolicySolver :
        public solvers::AROPolicySolverBase,
        public helpers::IndexedObjectOwner<SingleDirectionBreakPoints> {
public:
    using BreakPoint = SingleDirectionBreakPoints::BreakPoint;
    using BreakPointsSeries = SingleDirectionBreakPoints::BreakPointsSeries;
    using BreakPointDirection = SingleDirectionBreakPoints::BreakPointDirection;

public:

    explicit LiftingPolicySolver(ROModel const& original_model);

    void add_break_points(BreakPointsSeries const& break_points, BreakPointDirection const& break_point_direction);

    void set_use_dual_solver(bool use_dual_solver);

    void set_breakpoint_tightening(bool breakpoint_tightening);

    void add_equidistant_breakpoints(size_t num_pieces);

    //In case of symmetric uncertainty this will add num_pieces breakpoints on each side!
    void add_eta_induced_breakpoints(size_t num_pieces);

    void add_full_eta_induced_breakpoints();

    SolutionRealization specific_solution(std::vector<double> const& uncertainty_realization) const final;

    std::vector<double> lifted_uncertainty_realization(std::vector<double> const& uncertainty_realization) const;

private:

    void solve_implementation() final;

    void build_implementation() final;

    void build_axis_aligned_model();

    void add_decision_variables();

    void axis_aligned_lifted_model_add_lifted_uncertainty_variables();

    void axis_aligned_lifted_model_add_retracted_uncertainty_constraints();

    void axis_aligned_lifted_model_add_retracted_constraints();

    void axis_aligned_lifted_model_add_retracted_objective();

    void add_box_tightening_constraints();

    void add_rotational_invariant_tightening_constraints();

    double max_rotational_invariant_outside_budget(double bound) const;

    std::pair<std::vector<double>, std::vector<double>> get_box_bounds(
            std::vector<double> lbs, std::vector<double> ubs,
            UncertaintySetConstraintsSet::Index const& box_constraint_set) const;

    void add_box_tightening_constraint_in_direction(
            double box_lb, double box_ub, SingleDirectionBreakPoints const& break_point_series,
            UncertaintySetConstraintsSet::Index const& lifted_constraint_set
            );

    BreakPointsSeries cleaned_break_points(BreakPointsSeries const& break_points, BreakPointDirection const& break_point_direction) const;


    struct CutEncoding{
        size_t critical_layer; // corresponds to j in paper
        std::vector<size_t> active_indices; // corresponds to pi(1), ..., pi(i) in paper
    };

    bool add_most_violated_cut();

    bool add_most_violated_cut(
            size_t constr_id,
            UncertaintySetConstraintsSet::Index const& constraint_set);

    size_t num_positive_quadrant_pieces() const;

    AffineExpression<UncertaintyVariable::Reference> positive_quadrant_uncertainty_expressions(
            size_t positive_quadrant_piece,
            UncertaintyVariable::Index const& uvar) const;

    double positive_quadrant_break_point(size_t positive_quadrant_piece) const;

    double max_one_norm_k_active(size_t k) const;

    CutEncoding find_most_violated_cut(
            AffineAdjustablePolicyDualSolver::ScaledUncertainties const& scaled_uncertainties
            ) const;

    bool all_rotational_invariant_axis_aligned_breakpoints() const;

    bool all_symmetric_axis_aligned_breakpoints() const;

    ROModel & lifted_model();
    ROModel const& lifted_model() const;
    solvers::AROPolicySolverBase & affine_model();
    solvers::AROPolicySolverBase const& affine_model() const;
    AffineAdjustablePolicySolver & affine_primal_model();
    AffineAdjustablePolicySolver const& affine_primal_model() const;
    AffineAdjustablePolicyDualSolver & affine_dual_model();
    AffineAdjustablePolicyDualSolver const& affine_dual_model() const;

private:
    bool _all_simple_axis_aligned = true;
    bool _breakpoint_tightening = true;

    bool _use_dual_solver = false;

    double const cut_violation_threshold = 1e-1;
    double const improvement_threshold = 1e-5;
    double const breakpoint_distance_threshold = 1e-3;

    std::unique_ptr<AffineAdjustablePolicySolver> _affine_primal_model;
    std::unique_ptr<AffineAdjustablePolicyDualSolver> _affine_dual_model;
//    std::unique_ptr<solvers::AROPolicySolverBase> _affine_model;

    ROModel _lifted_model;
    std::vector<DecisionVariable::Reference> _lifted_decision_variables;
    std::vector<std::vector<UncertaintyVariable::Reference>> _lifted_uncertainty_variables;
    std::vector<AffineExpression<UncertaintyVariable::Reference>> _lifted_uncertainty_retractions;
};

}

#endif //ROBUSTOPTIMIZATION_LIFTINGPOLICYSOLVER_H
