#ifndef PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_UNCERTAINTYSAMPLER_H
#define PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_UNCERTAINTYSAMPLER_H

#include <memory>

#include "../basic_model_objects/UncertaintyVariable.h"
#include "../basic_model_objects/SOCExpression.h"

namespace robust_model {

class UncertaintySet;

class UncertaintySamplerBase {
public:
    explicit UncertaintySamplerBase(UncertaintySet const& uncertainty_set);
    virtual ~UncertaintySamplerBase() = default;

    virtual std::vector<std::vector<double>> sample(size_t num_realizations) const = 0;

    virtual std::vector<std::vector<double>> sample_tree(size_t num_children) const = 0;

    UncertaintySet const& uncertainty_set() const;

private:
    UncertaintySet const& _uncertainty_set;
};


class ExtendFromPartialUncertaintySamplerBase : public UncertaintySamplerBase {
public:
    explicit ExtendFromPartialUncertaintySamplerBase(UncertaintySet const& uncertainty_set);

    std::vector<std::vector<double>> sample(size_t num_realizations) const final;

    std::vector<std::vector<double>> sample_tree(size_t num_children) const final;


private:
    virtual std::vector<std::vector<double>> sample_partial(
            size_t num_children,
            size_t num_fixed_dimensions,
            std::vector<std::vector<double>> const& partial_realizations) const = 0;
};


class UniformBoxRejectUncertaintySampler : public ExtendFromPartialUncertaintySamplerBase {
public:
    explicit UniformBoxRejectUncertaintySampler(UncertaintySet const& uncertainty_set);

private:
    std::vector<std::vector<double>> sample_partial(
            size_t num_children, size_t num_fixed_dimensions,
            std::vector<std::vector<double>> const& partial_realizations) const final;
};

class UniformL2BallRejectUncertaintySampler : public ExtendFromPartialUncertaintySamplerBase {
public:
    UniformL2BallRejectUncertaintySampler(
            UncertaintySet const& uncertainty_set, double radius, bool non_negative);

private:
    std::vector<std::vector<double>> sample_partial(
            size_t num_children, size_t num_fixed_dimensions,
            std::vector<std::vector<double>> const& partial_realizations) const override;

    double radius() const;

    bool non_negative() const;

private:
    double const _radius;
    bool const _non_negative;
};

class UniformL1BallRejectUncertaintySampler : public ExtendFromPartialUncertaintySamplerBase {
public:
    UniformL1BallRejectUncertaintySampler(
            UncertaintySet const& uncertainty_set, double radius, bool non_negative);

private:
    std::vector<std::vector<double>> sample_partial(
            size_t num_children, size_t num_fixed_dimensions,
            std::vector<std::vector<double>> const& partial_realizations) const override;

    double radius() const;

    bool non_negative() const;

private:
    double const _radius;
    bool const _non_negative;
};

class TransformedUncertaintySampler : public UncertaintySamplerBase {
public:
    TransformedUncertaintySampler(
            UncertaintySet const& uncertainty_set,
            UncertaintySamplerBase const& base_uncertainty_sampler,
            std::vector<SOCExpression<UncertaintyVariable>> transformation);

    std::vector<std::vector<double>> sample(size_t num_realizations) const final;

    std::vector<std::vector<double>> sample_tree(size_t num_children) const final;

private:
    UncertaintySamplerBase const& base_uncertainty_sampler() const;

    std::vector<std::vector<double> > transform(std::vector<std::vector<double> > const& samples) const;

private:
    UncertaintySamplerBase const& _base_uncertainty_sampler;
    std::vector<SOCExpression<UncertaintyVariable>> const _transformation;
};

class OwningTransformedUncertaintySampler : public TransformedUncertaintySampler {
public:
    OwningTransformedUncertaintySampler(
            UncertaintySet const& uncertainty_set,
            std::unique_ptr<UncertaintySet> base_uncertainty_set,
            std::unique_ptr<UncertaintySamplerBase> base_uncertainty_sampler,
            std::vector<SOCExpression<UncertaintyVariable>> transformation
    );


private:
    std::unique_ptr<UncertaintySet> const _base_uncertainty_set;
    std::unique_ptr<UncertaintySamplerBase> const _base_uncertainty_sampler;
};

}

#endif //PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_UNCERTAINTYSAMPLER_H
