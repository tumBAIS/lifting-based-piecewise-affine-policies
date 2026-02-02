#include "AROPolicySolverBase.h"

namespace solvers {

AROPolicySolverBase::AROPolicySolverBase(robust_model::ROModel const& model) : _model(model) {}

robust_model::ROModel const& AROPolicySolverBase::model() const {
    return _model;
}


}
