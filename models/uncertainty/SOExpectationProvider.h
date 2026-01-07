#ifndef PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_SOEXPECTATIONPROVIDER_H
#define PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_SOEXPECTATIONPROVIDER_H

#include <functional>
#include <vector>

namespace robust_model {

class UncertaintyRealization;

class SOExpectationProvider {
public:
    virtual ~SOExpectationProvider() = default;
    virtual double expected_value(std::function<double(UncertaintyRealization const&)> const& fct) const = 0;
    virtual std::vector<double> expected_value(std::function<std::vector<double>(UncertaintyRealization const&)> const& fct) const = 0;
    virtual std::vector<std::vector<double>> expected_value(std::function<std::vector<std::vector<double>>(UncertaintyRealization const&)> const& fct) const = 0;
};

class SOExpectationProviderEmpirical : public SOExpectationProvider {
public:
    explicit SOExpectationProviderEmpirical(std::vector<UncertaintyRealization> empirical_uncertainty_realizations);
    explicit SOExpectationProviderEmpirical(std::vector<std::vector<double>> empirical_uncertainty_realizations);

    double expected_value(std::function<double(UncertaintyRealization const&)> const& fct) const final;
    std::vector<double> expected_value(std::function<std::vector<double>(UncertaintyRealization const&)> const& fct) const final;
    std::vector<std::vector<double>> expected_value(std::function<std::vector<std::vector<double>>(UncertaintyRealization const&)> const& fct) const override;


private:
    static std::vector<UncertaintyRealization> convert_to_realization_vector(
            std::vector<std::vector<double>> empirical_uncertainty_realizations);

private:
    std::vector<UncertaintyRealization> const _empirical_uncertainty_realizations;
};

}

#endif //PIECEWISEAFFINEADJUSTABLEOPTIMIZATION_SOEXPECTATIONPROVIDER_H
