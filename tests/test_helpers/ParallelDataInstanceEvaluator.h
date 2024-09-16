#ifndef ROBUSTOPTIMIZATION_PARALLELDATAINSTANCEEVALUATOR_H
#define ROBUSTOPTIMIZATION_PARALLELDATAINSTANCEEVALUATOR_H

#include <functional>
#include "../../helpers/helpers.h"

namespace data_models {

template<class Parameter>
struct ParameterIterator {
public:
    virtual bool increment() = 0;
    virtual Parameter get_parameter() const = 0;

    bool valid() const {
        return _valid;
    }
    void invalidate() {
        _valid = false;
    }

private:
    bool _valid = true;
};

template<class Parameter>
class ParallelDataInstanceEvaluator {
public:
    ParallelDataInstanceEvaluator(
            ParameterIterator<Parameter> & parameter_iterator,
            std::function<std::string (Parameter const&)> const& test,
            std::ostream& output_stream
    );

    void run_tests(size_t num_threads = 1);

private:
    void thread_function();

private:
    ParameterIterator<Parameter> & _parameter_iterator;
    std::mutex _increment_lock;
    std::function<std::string (Parameter const&)> const _test;
    std::mutex _output_lock;
    std::ostream& _output_stream;
};

}

#include "ParallelDataInstanceEvaluator.tplt"

#endif //ROBUSTOPTIMIZATION_PARALLELDATAINSTANCEEVALUATOR_H
