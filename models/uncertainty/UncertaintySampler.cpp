#include "UncertaintySet.h"
#include "UncertaintySampler.h"

#include <random>


namespace robust_model {


UncertaintySamplerBase::UncertaintySamplerBase(UncertaintySet const& uncertainty_set) :
        _uncertainty_set(uncertainty_set) {}

UncertaintySet const& UncertaintySamplerBase::uncertainty_set() const {
    return _uncertainty_set;
}

ExtendFromPartialUncertaintySamplerBase::ExtendFromPartialUncertaintySamplerBase(UncertaintySet const& uncertainty_set)
        : UncertaintySamplerBase(uncertainty_set) {}

std::vector<std::vector<double>> ExtendFromPartialUncertaintySamplerBase::sample(size_t num_realizations) const {
    return sample_partial(num_realizations, 0, {{}});
}

std::vector<std::vector<double>> ExtendFromPartialUncertaintySamplerBase::sample_tree(size_t num_children) const {
    std::vector<std::vector<double>> realizations(1, std::vector<double>{});
    period_id period = std::numeric_limits<int>::min();
    size_t fixed_vars = 0;
    for (auto const& uvar: uncertainty_set().variables()) {
        if (uvar.period() > period) {
            period = uvar.period();
            sample_partial(num_children, fixed_vars, realizations);
        }
        helpers::exception_check(uvar.period() == period, "Variables have to be ordered by period for tree sampling!");
        fixed_vars++;
    }
    return realizations;
}


UniformBoxRejectUncertaintySampler::UniformBoxRejectUncertaintySampler(UncertaintySet const& uncertainty_set)
        : ExtendFromPartialUncertaintySamplerBase(uncertainty_set) {}


std::vector<std::vector<double>>
UniformBoxRejectUncertaintySampler::sample_partial(
        size_t num_realizations_per_partial,
        size_t num_fixed_dimensions,
        std::vector<std::vector<double>> const& partial_realizations) const {

    std::ranlux48 generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> uniform_distribution(0., 1.);

    std::vector<std::vector<double>> realizations;
    realizations.reserve(num_realizations_per_partial * partial_realizations.size());

    bool previous_in_set = true;
    for (auto const& partial_realization: partial_realizations) {
        for (size_t i = 0; i < num_realizations_per_partial; i += previous_in_set) {
            if (previous_in_set) {
                realizations.emplace_back(std::vector<double>(uncertainty_set().num_variables()));
                for (size_t j = 0; j < num_fixed_dimensions; ++j) {
                    realizations.back()[j] = partial_realization[j];
                }
            }
            auto& realization = realizations.back();
            for (size_t j = num_fixed_dimensions; j < uncertainty_set().num_variables(); ++j) {
                auto const& uvar = uncertainty_set().variables()[j];
                double rn = uvar.lb() + (uvar.ub() - uvar.lb()) * uniform_distribution(generator);
                realization[j] = rn;
            }
            previous_in_set = uncertainty_set().in_uncertainty_set(UncertaintyRealization(realization));
        }
    }
    helpers::exception_check(previous_in_set, "When done, the last realization should be valid!");
    return realizations;
}

UniformL2BallRejectUncertaintySampler::UniformL2BallRejectUncertaintySampler(UncertaintySet const& uncertainty_set,
                                                                             double radius,
                                                                             bool non_negative)
        : ExtendFromPartialUncertaintySamplerBase(uncertainty_set), _radius(radius), _non_negative(non_negative) {
    helpers::exception_check(_radius > 0, "Need non-negative radius!");
}

std::vector<std::vector<double>>
UniformL2BallRejectUncertaintySampler::sample_partial(size_t num_children, size_t num_fixed_dimensions,
                                                      std::vector<std::vector<double>> const& partial_realizations) const {
    std::ranlux48 generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<double> normal_distribution(0., 1.);
    std::uniform_real_distribution<double> uniform_distribution(0., 1.);

    std::vector<std::vector<double>> realizations;
    realizations.reserve(num_children * partial_realizations.size());

    bool previous_in_set = true;
    for (auto const& partial_realization: partial_realizations) {
        double const reduced_radius = [&]() {
            double square_norm = 0;
            for (size_t j = 0; j < num_fixed_dimensions; ++j) {
                square_norm += partial_realization[j] * partial_realization[j];
            }
            return std::sqrt(radius() * radius() - square_norm);
        }();
        for (size_t i = 0; i < num_children; i += previous_in_set) {
            if (previous_in_set) {
                realizations.emplace_back(std::vector<double>(uncertainty_set().num_variables()));
                for (size_t j = 0; j < num_fixed_dimensions; ++j) {
                    realizations.back()[j] = partial_realization[j];
                }
            }
            auto& realization = realizations.back();
            double norm = 0;
            for (size_t j = num_fixed_dimensions; j < uncertainty_set().num_variables(); ++j) {
                double rn = non_negative() ? std::abs(normal_distribution(generator)) : normal_distribution(generator);
                realization[j] = rn;
                norm += rn * rn;
            }
            norm = std::sqrt(norm);
            double const radius =
                    std::pow(uniform_distribution(generator), 1. / double(uncertainty_set().num_variables())) *
                    reduced_radius;
            double const scale = radius / norm;
            for (size_t j = num_fixed_dimensions; j < uncertainty_set().num_variables(); ++j) {
                realization[j] *= scale;
            }
            previous_in_set = uncertainty_set().in_uncertainty_set(UncertaintyRealization(realization));
        }
    }
    helpers::exception_check(previous_in_set, "When done, the last realization should be valid!");
    return realizations;
}

double UniformL2BallRejectUncertaintySampler::radius() const {
    return _radius;
}

bool UniformL2BallRejectUncertaintySampler::non_negative() const {
    return _non_negative;
}

UniformL1BallRejectUncertaintySampler::UniformL1BallRejectUncertaintySampler(UncertaintySet const& uncertainty_set,
                                                                             double radius,
                                                                             bool non_negative)
        : ExtendFromPartialUncertaintySamplerBase(uncertainty_set), _radius(radius), _non_negative(non_negative) {
    helpers::exception_check(_radius > 0, "Need non-negative radius!");
}

std::vector<std::vector<double>>
UniformL1BallRejectUncertaintySampler::sample_partial(size_t num_children, size_t num_fixed_dimensions,
                                                      std::vector<std::vector<double>> const& partial_realizations) const {
    std::ranlux48 generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::exponential_distribution<double> exponential_distribution(1.);
    std::uniform_int_distribution<> sign_generator(0, 1);

    std::vector<std::vector<double>> realizations;
    realizations.reserve(num_children * partial_realizations.size());

    bool previous_in_set = true;
    for (auto const& partial_realization: partial_realizations) {
        double const reduced_radius = [&]() {
            double one_norm = 0;
            for (size_t j = 0; j < num_fixed_dimensions; ++j) {
                one_norm += std::abs(partial_realization[j]);
            }
            return std::sqrt(radius() - one_norm);
        }();
        for (size_t i = 0; i < num_children; i += previous_in_set) {
            if (previous_in_set) {
                realizations.emplace_back(std::vector<double>(uncertainty_set().num_variables()));
                for (size_t j = 0; j < num_fixed_dimensions; ++j) {
                    realizations.back()[j] = partial_realization[j];
                }
            }
            auto& realization = realizations.back();
            double norm = exponential_distribution(generator);
            for (size_t j = 0; j < uncertainty_set().num_variables(); ++j) {
                double rn = exponential_distribution(generator);
                realization[j] = rn * ((non_negative() or sign_generator(generator) == 1) ? 1. : -1.);
                norm += rn;
            }
            double const scale = reduced_radius / norm;
            for (size_t j = 0; j < uncertainty_set().num_variables(); ++j) {
                realization[j] *= scale;
            }
            previous_in_set = uncertainty_set().in_uncertainty_set(UncertaintyRealization(realization));
        }
    }
    helpers::exception_check(previous_in_set, "When done, the last realization should be valid!");
    return realizations;
}

double UniformL1BallRejectUncertaintySampler::radius() const {
    return _radius;
}

bool UniformL1BallRejectUncertaintySampler::non_negative() const {
    return _non_negative;
}

TransformedUncertaintySampler::TransformedUncertaintySampler(
        UncertaintySet const& uncertainty_set,
        UncertaintySamplerBase const& base_uncertainty_sampler,
        std::vector<SOCExpression<UncertaintyVariable>> transformation) :
        UncertaintySamplerBase(uncertainty_set),
        _base_uncertainty_sampler(base_uncertainty_sampler),
        _transformation(std::move(transformation)) {
    helpers::exception_check(_transformation.size() == uncertainty_set.num_variables(),
                             "Provide Transformation for each Variable!");
}

std::vector<std::vector<double>> TransformedUncertaintySampler::sample(size_t num_realizations) const {
    return transform(base_uncertainty_sampler().sample(num_realizations));
}

std::vector<std::vector<double>> TransformedUncertaintySampler::sample_tree(size_t num_children) const {
    return transform(base_uncertainty_sampler().sample_tree(num_children));
}

UncertaintySamplerBase const& TransformedUncertaintySampler::base_uncertainty_sampler() const {
    return _base_uncertainty_sampler;
}

std::vector<std::vector<double> >
TransformedUncertaintySampler::transform(std::vector<std::vector<double>> const& samples) const {
    std::vector<std::vector<double>> transformed_samples;
    transformed_samples.reserve(samples.size());
    for (auto const& sample: samples) {
        std::vector<double> transformed_sample;
        transformed_sample.reserve(sample.size());
        UncertaintyRealization const sample_realization(sample);

        for (auto const& transformation: _transformation) {
            transformed_sample.emplace_back(transformation.value(sample_realization));
        }
        transformed_samples.emplace_back(transformed_sample);
    }
    return transformed_samples;
}

OwningTransformedUncertaintySampler::OwningTransformedUncertaintySampler(
        UncertaintySet const& uncertainty_set,
        std::unique_ptr<UncertaintySet> base_uncertainty_set,
        std::unique_ptr<UncertaintySamplerBase> base_uncertainty_sampler,
        std::vector<SOCExpression<UncertaintyVariable>> transformation
) : TransformedUncertaintySampler(uncertainty_set, *base_uncertainty_sampler, std::move(transformation)),
    _base_uncertainty_set(std::move(base_uncertainty_set)),
    _base_uncertainty_sampler(std::move(base_uncertainty_sampler)) {
    helpers::exception_check(_base_uncertainty_set.get() == &_base_uncertainty_sampler->uncertainty_set(),
                             "Base sampler has to sample from base uncertainty set!");
}


}
