#ifndef ROBUSTOPTIMIZATION_DATADRIVENLIFTEDINVENTORYMANAGEMENTCVMODEL_H
#define ROBUSTOPTIMIZATION_DATADRIVENLIFTEDINVENTORYMANAGEMENTCVMODEL_H

#include "DataModelBase.h"
#include "../../helpers/helpers.h"

namespace data_models {

template<class BaseModel>
class CVModel : public DataModelBase {
public:
    CVModel(typename BaseModel::Parameter parameter, std::vector<double>  radii, size_t cv_split = 5);

    bool train(SampleData const& training_data) final;

    ScoreOutput test(SampleData const& test_data) const final;

    double train_time() const final;

    double best_radius() const;

private:
    std::tuple<SampleData, SampleData> validation_split(size_t cv_seed, SampleData const& training_data) const;

private:
    typename BaseModel::Parameter const _parameter;
    std::vector<double> const _radii;
    size_t const _cv_split;

    double _train_time = 0;

    std::optional<double> _best_radius;
    std::unique_ptr<BaseModel> _best_model;
};

}

#include "CVModel.tplt"

#endif //ROBUSTOPTIMIZATION_DATADRIVENLIFTEDINVENTORYMANAGEMENTCVMODEL_H
