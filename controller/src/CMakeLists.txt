add_library(source2 pid.cpp lqr.cpp riccati_solver.cpp)
target_include_directories(source2 PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
