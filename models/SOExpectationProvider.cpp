#include "SOExpectationProvider.h"
#include "UncertaintySet.h"


namespace robust_model {


SOExpectationProviderEmpirical::SOExpectationProviderEmpirical(
        std::vector<UncertaintyRealization> empirical_uncertainty_realizations) :
        _empirical_uncertainty_realizations(std::move(empirical_uncertainty_realizations)) {}

SOExpectationProviderEmpirical::SOExpectationProviderEmpirical(
        std::vector<std::vector<double>> empirical_uncertainty_realizations) :
        SOExpectationProviderEmpirical(
                convert_to_realization_vector(std::move(empirical_uncertainty_realizations))
        ) {}

double SOExpectationProviderEmpirical::expected_value(
        std::function<double(UncertaintyRealization const&)> const& fct) const {
    double ev = 0;
    for (auto const& realization: _empirical_uncertainty_realizations) {
        ev += fct(realization);
    }
    return ev / double(_empirical_uncertainty_realizations.size());
}

std::vector<double> SOExpectationProviderEmpirical::expected_value(
        std::function<std::vector<double>(UncertaintyRealization const&)> const& fct) const {
    std::vector<double> ev;
    for (auto const& realization: _empirical_uncertainty_realizations) {
        auto res = fct(realization);
        if (ev.empty())
            ev = std::move(res);
        else
            for (size_t i = 0; i < ev.size(); ++i) {
                ev.at(i) += res.at(i);
            }
    }
    for (auto& val: ev) {
        val /= double(_empirical_uncertainty_realizations.size());
    }
    return ev;
}

std::vector<std::vector<double>> SOExpectationProviderEmpirical::expected_value(
        std::function<std::vector<std::vector<double>>(UncertaintyRealization const&)> const& fct) const {
    std::vector<std::vector<double>> ev;
    for (auto const& realization: _empirical_uncertainty_realizations) {
        auto res = fct(realization);
        if (ev.empty())
            ev = std::move(res);
        else
            for (size_t i = 0; i < ev.size(); ++i) {
                for (size_t j = 0; j < ev.at(i).size(); ++j) {
                    ev.at(i).at(j) += res.at(i).at(j);
                }
            }
    }
    for (auto& row: ev) {
        for (auto& val: row) {
            val /= double(_empirical_uncertainty_realizations.size());
        }
    }
    return ev;
}


std::vector<UncertaintyRealization> SOExpectationProviderEmpirical::convert_to_realization_vector(
        std::vector<std::vector<double>> empirical_uncertainty_realizations) {
    std::vector<UncertaintyRealization> realizations;
    for (auto& realization: empirical_uncertainty_realizations) {
        realizations.emplace_back(std::move(realization));
    }
    return realizations;
}


}