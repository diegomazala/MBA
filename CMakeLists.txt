# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(multivariate_b-splines VERSION 0.1 LANGUAGES CXX)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_MODULE_PATH              ${PROJECT_SOURCE_DIR}/cmake/Modules)

find_package(OpenMesh REQUIRED)

# Targets that we develop here
#enable_testing()
add_subdirectory(src/mba)
add_subdirectory(src/app)


