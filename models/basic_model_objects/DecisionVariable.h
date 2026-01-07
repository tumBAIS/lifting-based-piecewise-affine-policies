#ifndef ROBUSTOPTIMIZATION_DECISIONVARIABLE_H
#define ROBUSTOPTIMIZATION_DECISIONVARIABLE_H

#include <optional>
#include <map>
#include <functional>

#include "../../helpers/helpers.h"
#include "types_and_constants.h"
#include "VariableBase.h"
#include "UncertaintyVariable.h"

namespace robust_model {

class SolutionRealization;

struct DominatingSolution {
    explicit DominatingSolution(std::vector<double> solutions);

    std::vector<double> const& values_per_vertex() const;

    std::string to_string() const;

private:
    std::vector<double> _values_per_vertex;
};

struct AffineSolution {
    explicit AffineSolution(double constant,
                            std::map<UncertaintyVariable::Index, double> dependent_scales);

    double constant() const;

    std::map<UncertaintyVariable::Index, double> const& dependent_scales() const;

    std::string to_string() const;

private:
    double _constant;
    std::map<UncertaintyVariable::Index, double> _dependent_scales;
};

class DecisionVariable :
        public VariableBase<DecisionVariable> {
public:
    class Dependency {
    public:
        struct Index {
            explicit Index(size_t raw_id) : _raw_id(raw_id) {}

            size_t raw_id() const { return _raw_id; };
        private:
            size_t const _raw_id;
        };

        Dependency(
                Index id,
                DecisionVariable::Reference decision_variable,
                UncertaintyVariable::Reference uncertainty_variable);

        Index id() const;

        DecisionVariable::Reference decision_variable() const;

        UncertaintyVariable::Reference uncertainty_variable() const;

    private:
        Index const _id;
        DecisionVariable::Reference const _decision_variable;
        UncertaintyVariable::Reference const _uncertainty_variable;
    };

public:
    DecisionVariable(Index id, std::string const& name,
                     std::optional<period_id> p,
                     double lb, double ub);

    void add_dependency(UncertaintyVariable::Index dependency);

    void add_dependencies(std::vector<UncertaintyVariable::Reference> const& dependencies);

    std::vector<Dependency> const& dependencies() const;

    size_t num_dependencies() const;

    bool has_period() const;

    period_id period() const;

    AffineSolution const& affine_solution() const;

    void set_affine_solution(AffineSolution const& affine_solution);

    DominatingSolution const& dominating_solution() const;

    void set_dominating_solution(DominatingSolution const& dominating_solution);

    void set_exact_evaluation(std::function<double (SolutionRealization const&)> const& evaluation);
    bool has_exact_evaluation() const;
    double exact_evaluation(SolutionRealization const& realization) const;

private:
    std::vector<Dependency> _dependencies;
    std::optional<period_id> const _period;
    std::optional<std::function<double (SolutionRealization const &)> >_exact_evaluation;
    std::optional<AffineSolution> _affine_solution;
    std::optional<DominatingSolution> _dominating_solution;
};

}

#endif //ROBUSTOPTIMIZATION_DECISIONVARIABLE_H
