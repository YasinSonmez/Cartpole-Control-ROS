add_library(source inverted_pendulum.cpp)
target_include_directories(source PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
