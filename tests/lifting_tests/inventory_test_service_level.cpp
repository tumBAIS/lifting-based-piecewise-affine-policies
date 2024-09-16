#include "robust_inventory/MultistageInventoryManagementInstanceGeneratorServiceLevel.h"
#include "../../solvers/aro_policy_solvers/LiftingPolicySolver.h"
#include "../test_helpers/ParallelInstanceEvaluator.h"

int
main(int argc,
     char *argv[]) {
    std::string run_name = "inventory_test_service_level_" + helpers::time_stamp();
    helpers::global_logger.set_logfile("../logs/" + run_name + ".log");
    helpers::global_logger << "Logging " + run_name;
    std::ofstream output_stream("../results/" + run_name + ".csv");
//    auto & output_stream = std::cout;
    auto instance_generator = testing::MultistageInventoryManagementInstanceGeneratorServiceLevel();
    auto tester = testing::ParallelInstanceEvaluator(output_stream, instance_generator);

    double const max_runtime = 259200;

    tester.add_test("AFF",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto am = robust_model::AffineAdjustablePolicySolver(model);
                        am.build();
                        am.set_runtime_limit(max_runtime);
                        am.solve();
                        return std::make_tuple(am.runtime(), am.objective_value());
                    });
    tester.add_test("GLIFT1",
                    [max_runtime](robust_model::ROModel const& model) {
                        // For non-stochastic model this is provably the same as AFF
                        if (model.objective().expression().uncertainty_behaviour() !=
                            robust_model::RoAffineExpression::UncertaintyBehaviour::STOCHASTIC) {
                            return std::make_tuple(std::nan("0"), std::nan("0"));
                        }
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_equidistant_breakpoints(2);
                        lm.set_breakpoint_tightening(false);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });
    tester.add_test("GLIFT3",
                    [max_runtime](robust_model::ROModel const& model) {
                        // For non-stochastic model this is provably the same as AFF
                        if (model.objective().expression().uncertainty_behaviour() !=
                            robust_model::RoAffineExpression::UncertaintyBehaviour::STOCHASTIC) {
                            return std::make_tuple(std::nan("0"), std::nan("0"));
                        }
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_equidistant_breakpoints(4);
                        lm.set_breakpoint_tightening(false);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });
    tester.add_test("GLIFTF",
                    [max_runtime](robust_model::ROModel const& model) {
                        // For non-stochastic model this is provably the same as AFF
                        if (model.objective().expression().uncertainty_behaviour() !=
                            robust_model::RoAffineExpression::UncertaintyBehaviour::STOCHASTIC) {
                            return std::make_tuple(std::nan("0"), std::nan("0"));
                        }
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_full_kappa_induced_breakpoints();
                        lm.set_breakpoint_tightening(false);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });
    tester.add_test("LIFT1",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_equidistant_breakpoints(2);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });
    tester.add_test("LIFT3",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_equidistant_breakpoints(4);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });
    tester.add_test("LIFTF",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_full_kappa_induced_breakpoints();
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });


    for (size_t i = 1; i <= 4; ++i) {
        instance_generator.add_num_stages(5 * i);
    }
    for (double alpha: {0., .25, .5}) {
        instance_generator.add_alpha(alpha);
    }
    for (double service_level: {.05}) {
        instance_generator.add_service_level(service_level);
    }

    instance_generator.add_set_type(robust_model::UncertaintySet::SpecialSetType::BALL);
//    instance_generator.add_budget([](size_t num_u){return sqrt(sqrt(double(num_u)));}, "sqrtsqrt");


    instance_generator.set_number_of_iterations(1);

    tester.run_tests(6);
}