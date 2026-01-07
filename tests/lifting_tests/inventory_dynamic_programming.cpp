#include <cmath>
#include <numbers>
#include <vector>
#include <iostream>

#include "../test_helpers/ParallelDataInstanceEvaluator.h"
#include "dp_inventory/DPInventorySolver.h"

struct Parameter {
    size_t T;
    double alpha;
    double discretization;
    double const_replenishment;
    bool robust;

    std::string csv_string() const {
        return std::to_string(T) + "," +
               std::to_string(alpha) + "," +
               std::to_string(discretization) + "," +
               std::to_string(robust) + "," +
               std::to_string(const_replenishment);
    }
};

class ParameterIterator : public data_models::ParameterIterator<Parameter> {
public:
    bool increment() override {
        if (not valid())
            return false;

        if (++_alpha_options_it < _alpha_options.size())
            return true;
        _alpha_options_it = 0;
        if (++_const_replenishment_options_it < _const_replenishment_options.size())
            return true;
        _const_replenishment_options_it = 0;
        if (++_discretization_options_it < _discretization_options.size())
            return true;
        _discretization_options_it = 0;
        if (not _robust) {
            _robust = true;
            return true;
        }
        _robust = false;
        if (++_stage_num_options_it < _stage_num_options.size())
            return true;
        _stage_num_options_it = 0;

        invalidate();
        return false;
    }

    Parameter get_parameter() const override {
        return {_stage_num_options.at(_stage_num_options_it),
                _alpha_options.at(_alpha_options_it),
                _discretization_options.at(_discretization_options_it),
                _const_replenishment_options.at(_const_replenishment_options_it),
                _robust
        };
    }

private:
    std::vector<size_t> const _stage_num_options = {5, 10, 15, 20};
    size_t _stage_num_options_it = 0;
    std::vector<double> const _alpha_options = {0., .25, .5};
    size_t _alpha_options_it = 0;
    std::vector<double> const _discretization_options = {5.};
    size_t _discretization_options_it = 0;
    std::vector<double> const _const_replenishment_options = {200.};
    size_t _const_replenishment_options_it = 0;
    bool _robust = false;
};

double dp_test(Parameter const& parameter) {
    testing::DPInventorySolver solver(
            parameter.T, parameter.alpha, parameter.discretization, parameter.robust, parameter.const_replenishment);
    return solver.compute_value();
}

std::string bin_search_dp_test(Parameter parameter) {
    auto const discretization = parameter.discretization;
    auto const x0 = parameter.const_replenishment;

    auto discretize = [&](double x) {
        return std::floor(x / discretization) * discretization;
    };

    double mid = discretize(x0);
    double low = discretize(.8 * x0);
    double up = discretize(1.2 * x0);
    double fmid = dp_test(parameter);

    while (up - low > 2 * discretization) {
        if (mid - low > up - mid) {
            double x = discretize((low + mid) / 2.0);
            parameter.const_replenishment = x;
            double fx = dp_test(parameter);
            if (fx < fmid) {
                up = mid;
                mid = x;
                fmid = fx;
            } else {
                low = x;
            }
        } else {
            double x = discretize((up + mid) / 2.0);
            parameter.const_replenishment = x;
            double fx = dp_test(parameter);
            if (fx < fmid) {
                low = mid;
                mid = x;
                fmid = fx;
            } else {
                up = x;
            }
        }
    }
    parameter.const_replenishment = mid;
    std::string ret = parameter.csv_string() + "," + std::to_string(fmid) + "\n";

    for (double disc: {2.5, 4., 5., 8., 10.}) {
        if (disc == discretization)
            continue;
        parameter.discretization = disc;
        ret += parameter.csv_string() + "," + std::to_string(dp_test(parameter)) + "\n";
    }
    return ret;
}

std::string grid_search_dp_test(Parameter parameter) {
    auto const discretization = parameter.discretization;
    auto const x0 = parameter.const_replenishment;

    auto discretize = [&](double x) {
        return std::floor(x / discretization) * discretization;
    };

    double low = discretize(.9 * x0);
    double up = discretize(1.2 * x0);

    double x = low;

    double xopt = x;
    double fxopt = std::numeric_limits<double>::infinity();

    while (x < up) {
        parameter.const_replenishment = x;
        double fx = dp_test(parameter);
        if (fx < fxopt){
            xopt = x;
            fxopt = fx;
        }
        x += discretization;
    }
    parameter.const_replenishment = xopt;
    std::string ret = parameter.csv_string() + "," + std::to_string(fxopt) + "\n";

    for (double disc: {2.5, 4., 5., 8., 10.}) {
        if (disc == discretization)
            continue;
        parameter.discretization = disc;
        ret += parameter.csv_string() + "," + std::to_string(dp_test(parameter)) + "\n";
    }
    return ret;
}

int
main(int argc,
     char *argv[]) {
    std::ofstream output_stream("../results/dp_test.csv");
    ParameterIterator iterator;
    data_models::ParallelDataInstanceEvaluator<Parameter> evaluator(iterator, grid_search_dp_test, output_stream);
    output_stream
            << "T,alpha,discretization,robust,const_replenishment,value"
            << std::endl;
    evaluator.run_tests(3);
}


