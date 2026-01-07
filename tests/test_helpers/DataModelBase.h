#ifndef ROBUSTOPTIMIZATION_DATAMODELBASE_H
#define ROBUSTOPTIMIZATION_DATAMODELBASE_H

#include "../../helpers/helpers.h"
#include "../../models/basic_model_objects/types_and_constants.h"

namespace data_models {

struct ScoreOutput {
    double mean_objective;
    double standard_deviation;
    double valid_fraction = 1;

    std::string to_string() const {
        return "AVG: " + std::to_string(mean_objective) +
        ", STD: " + std::to_string(standard_deviation) +
        ", VALID: " + std::to_string(valid_fraction);
    }
    std::string csv_string() const {
        return std::to_string(mean_objective) + ";" +
        std::to_string(standard_deviation) + ";" +
        std::to_string(valid_fraction);
    }
};

class DataModelBase {
public:
    using DataPoint = std::vector<double>;
    using SampleData = std::vector<DataPoint>;

public:
    virtual bool train(SampleData const& training_data) = 0;

    virtual ScoreOutput test(SampleData const& test_data) const = 0;

    virtual double train_time() const = 0;

    static
    std::tuple<std::vector<double>, std::vector<double>>
    get_uncertainty_bounds(DataModelBase::SampleData const& training_data) {
        size_t const sample_size = training_data.size();
        size_t const data_dimension = training_data.front().size();
        std::vector<double> lbs(data_dimension, robust_model::NO_VARIABLE_UB);
        std::vector<double> ubs(data_dimension, robust_model::NO_VARIABLE_LB);
        for (auto const& sample: training_data) {
            for (size_t i = 0; i < data_dimension; ++i) {
                lbs.at(i) = std::min(lbs.at(i), sample.at(i));
                ubs.at(i) = std::max(ubs.at(i), sample.at(i));
            }
        }
        return {lbs, ubs};
    }
};

}

#endif //ROBUSTOPTIMIZATION_DATAMODELBASE_H
