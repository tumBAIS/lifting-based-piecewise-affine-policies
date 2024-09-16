#ifndef ROBUSTOPTIMIZATION_DATAMODELBASE_H
#define ROBUSTOPTIMIZATION_DATAMODELBASE_H

#include "../../helpers/helpers.h"

namespace data_models {

struct ScoreOutput {
    double mean_objective;
    double standard_deviation;

    std::string to_string() const {
        return "AVG: " + std::to_string(mean_objective) + ", STD: " + std::to_string(standard_deviation);
    }
    std::string csv_string() const {
        return std::to_string(mean_objective) + "; " + std::to_string(standard_deviation);
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
};

}

#endif //ROBUSTOPTIMIZATION_DATAMODELBASE_H
