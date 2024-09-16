#ifndef ROBUSTOPTIMIZATION_DATADRIVENLIFTEDINVENTORYMANAGEMENTMODEL_H
#define ROBUSTOPTIMIZATION_DATADRIVENLIFTEDINVENTORYMANAGEMENTMODEL_H

#include "../../test_helpers/DataModelBase.h"
#include "../../../helpers/helpers.h"
#include "../../../solvers/aro_policy_solvers/LiftingPolicySolver.h"

namespace data_models {

class DataDrivenLiftedInventoryManagementModel : public DataModelBase {
public:
    enum class Mode {
        AFFINE,
        LIFTED_EQUIDISTANT,
        LIFTED_EQUIDISTANT_OLD,
        LIFTED_PERCENTILES
    };

    static std::string to_string(Mode mode) {
        switch (mode) {
            case Mode::AFFINE:
                return "AFF";
            case Mode::LIFTED_EQUIDISTANT:
                return "LIFTE";
            case Mode::LIFTED_EQUIDISTANT_OLD:
                return "LIFTEOLD";
            case Mode::LIFTED_PERCENTILES:
                return "LIFTP";
        }
    }

    struct Parameter {
        Mode const mode;
        double const overage_cost;
        double const eoh_underage_cost;
        size_t const num_lifting_parts;
    };

public:
    DataDrivenLiftedInventoryManagementModel(Parameter parameter, double radius);

    DataDrivenLiftedInventoryManagementModel(Mode mode, double radius, double overage_cost,
                                             double eoh_underage_cost,
                                             size_t num_lifting_parts = 0);

    bool train(SampleData const& training_data) final;

    ScoreOutput test(SampleData const& test_data) const final;

    double train_time() const final;

private:
    void build_ro_model(SampleData const& training_data);

    solvers::SolverBase const& active_solver() const;

    std::tuple<std::vector<double>, std::vector<double>> get_uncertainty_bounds(SampleData const& training_data) const;

    void solve_ro_model(SampleData const& training_data);

    robust_model::SolutionRealization realization(DataPoint const& sample) const;

    double realization_objective(robust_model::SolutionRealization const& realization) const;

    std::vector<robust_model::SingleDirectionBreakPoints::BreakPointsSeries> calculate_equidistant_percentiles(
            size_t num_pieces,
            SampleData const& training_data);

    std::vector<robust_model::SingleDirectionBreakPoints::BreakPointsSeries> calculate_percentiles(
            std::vector<double> percentiles,
            SampleData const& training_data);

private:
    Mode const _mode;
    double const _radius;
    size_t const _num_lifting_parts;

    std::unique_ptr<robust_model::AffineAdjustablePolicySolver> _affine_model;
    std::unique_ptr<robust_model::LiftingPolicySolver> _lifting_model;

    robust_model::ROModel _model;

    double const _max_order = 260;
    double const _order_cost = .1;
    double const _overage_cost;
    double const _underage_cost = .2;
    double const _underage_cost_last;
    double const _early_order_cost = 0.;

    std::vector<robust_model::DecisionVariable::Reference> _reordering_quantities;
    std::vector<robust_model::DecisionVariable::Reference> _early_ordering_quantities;
};

}

#endif //ROBUSTOPTIMIZATION_DATADRIVENLIFTEDINVENTORYMANAGEMENTMODEL_H
