#include "robust_inventory/MultistageInventoryManagementInstanceGenerator.h"
#include "../../solvers/aro_policy_solvers/LiftingPolicySolver.h"
#include "../test_helpers/ParallelInstanceEvaluator.h"

int
main(int argc,
     char *argv[]) {
    std::string run_name = "inventory_test_lifting_" + helpers::time_stamp();
    helpers::global_logger.set_logfile("../logs/" + run_name + ".log");
    helpers::global_logger << "Logging " + run_name;
    std::ofstream output_stream("../results/" + run_name + ".csv");
    auto instance_generator = testing::MultistageInventoryManagementInstanceGenerator();
    auto tester = testing::ParallelInstanceEvaluator(output_stream, instance_generator);

    double const max_runtime = 1800;

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
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_equidistant_breakpoints(4);
                        lm.set_breakpoint_tightening(false);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });

    tester.add_test("GLIFT3'",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_eta_induced_breakpoints(2);
                        lm.set_breakpoint_tightening(false);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });
    tester.add_test("GLIFTF",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_full_eta_induced_breakpoints();
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

    tester.add_test("LIFT3'",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_eta_induced_breakpoints(2);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });
    tester.add_test("LIFTF",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_full_eta_induced_breakpoints();
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });
    tester.add_test("LIFT1 CG",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.set_use_dual_solver(true);
                        lm.add_equidistant_breakpoints(2);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });
    tester.add_test("LIFT3 CG",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.set_use_dual_solver(true);
                        lm.add_equidistant_breakpoints(4);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });
    tester.add_test("LIFT3' CG",
                    [max_runtime](robust_model::ROModel const& model) {
                        auto lm = robust_model::LiftingPolicySolver(model);
                        lm.add_eta_induced_breakpoints(2);
                        lm.set_use_dual_solver(true);
                        lm.build();
                        lm.set_runtime_limit(max_runtime);
                        lm.solve();
                        return std::make_tuple(lm.runtime(), lm.objective_value());
                    });


    for (size_t i = 1; i <= 4; ++i) {
        instance_generator.add_num_stages(5 * i);
    }
    for (size_t i = 0; i <= 2; ++i) {
        instance_generator.add_alpha(.25 * double(i));
    }
    for (double cost: {.04}) {
        instance_generator.add_overage_cost(cost);
    }

    instance_generator.add_uncertainty_scale_function([](size_t T) { return std::sqrt(double(T)); }, "scale_sqrtT");

    instance_generator.add_end_of_horizon_scale_function([](size_t T) { return 10.; }, "scale_10");

    instance_generator.add_set_type(robust_model::UncertaintySet::SpecialSetType::OTHER);

    instance_generator.set_number_of_iterations(1);

    tester.run_tests(6);
}