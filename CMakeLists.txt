cmake_minimum_required(VERSION 3.16)
project(PiecewiseAffineAdjustableOptimization)

set(CMAKE_CXX_STANDARD 20)

# Find Threads Package
# Only needed for testing in parallel
find_package(Threads)

# Find Gurobi library
# Adjust Manually when needed
IF(EXISTS $ENV{GUROBI_HOME})
    MESSAGE("Gurobi path found!")
ELSE()
    MESSAGE("Gurobi path not found!")
ENDIF()

include_directories($ENV{GUROBI_HOME}/include)

# Might need to add the installed gurobi version to the list
find_library(GUROBI_LIBRARY NAMES
        NAMES gurobi91 gurobi95 gurobi100 gurobi110
        HINTS $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib)

find_library(CPP_GUROBI_LIBRARY NAMES
        NAMES gurobi_c++ gurobi_c++md2019
        HINTS $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib)

find_library(CPP_GUROBI_LIBRARY_DEBUG NAMES
        NAMES gurobi_c++ gurobi_c++mdd2019
        HINTS $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib)

message(${CPP_GUROBI_LIBRARY})


#########
# Helpers
#########

file(GLOB_RECURSE HELPERS_INCLUDE "helpers/*.h")
file(GLOB_RECURSE HELPERS_TEMPLATES "helpers/*.tplt")
file(GLOB_RECURSE HELPERS_SOURCES "helpers/*.cpp")
add_library(Helpers ${HELPERS_INCLUDE} ${HELPERS_TEMPLATES} ${HELPERS_SOURCES})


########
# Models
########

file(GLOB_RECURSE MODELS_INCLUDE "models/*.h")
file(GLOB_RECURSE MODELS_TEMPLATES "models/*.tplt")
file(GLOB_RECURSE MODELS_SOURCES "models/*.cpp")
add_library(Models ${MODELS_INCLUDE} ${MODELS_TEMPLATES} ${MODELS_SOURCES})
target_link_libraries(Models PRIVATE Helpers)


#########
# Solvers
#########

file(GLOB_RECURSE SOLVERS_INCLUDE "solvers/*.h")
file(GLOB_RECURSE SOLVERS_TEMPLATES "solvers/*.tplt")
file(GLOB_RECURSE SOLVERS_SOURCES "solvers/*.cpp")
add_library(Solvers ${SOLVERS_INCLUDE} ${SOLVERS_TEMPLATES} ${SOLVERS_SOURCES})
target_link_libraries(Solvers PRIVATE Helpers Models)
target_link_libraries(Solvers PRIVATE
        optimized ${CPP_GUROBI_LIBRARY}
        debug ${CPP_GUROBI_LIBRARY_DEBUG}
        general ${GUROBI_LIBRARY})


##############
# Test Helpers
##############

file(GLOB_RECURSE TEST_HELPERS_INCLUDE "tests/test_helpers/*.h")
file(GLOB_RECURSE TEST_HELPERS_TEMPLATES "tests/test_helpers/*.tplt")
file(GLOB_RECURSE TEST_HELPERS_SOURCES "tests/test_helpers/*.cpp")
add_library(TestHelpers ${TEST_HELPERS_INCLUDE} ${TEST_HELPERS_TEMPLATES} ${TEST_HELPERS_SOURCES})
target_link_libraries(TestHelpers PUBLIC Helpers Models Solvers)
target_link_libraries(TestHelpers ${CMAKE_THREAD_LIBS_INIT})


###############
# Lifting Tests
###############

# Robust Inventory Instances

file(GLOB_RECURSE ROBUST_INVENTORY_TEST_INCLUDE "tests/lifting_tests/robust_inventory/*.h")
file(GLOB_RECURSE ROBUST_INVENTORY_TEST_SOURCES "tests/lifting_tests/robust_inventory/*.cpp")
add_executable(RobustInventoryTestServiceLevel tests/lifting_tests/inventory_test_service_level.cpp
        ${ROBUST_INVENTORY_TEST_INCLUDE} ${ROBUST_INVENTORY_TEST_SOURCES})
target_link_libraries(RobustInventoryTestServiceLevel TestHelpers)

# Data Driven Inventory Instances

file(GLOB_RECURSE DATA_DRIVEN_INVENTORY_TEST_INCLUDE "tests/lifting_tests/data_driven_inventory/*.h")
file(GLOB_RECURSE DATA_DRIVEN_INVENTORY_TEST_SOURCES "tests/lifting_tests/data_driven_inventory/*.cpp")
add_executable(DataDrivenInventoryTest tests/lifting_tests/data_driven_inventory_test.cpp
        ${DATA_DRIVEN_INVENTORY_TEST_INCLUDE} ${DATA_DRIVEN_INVENTORY_TEST_SOURCES})
target_link_libraries(DataDrivenInventoryTest TestHelpers)

