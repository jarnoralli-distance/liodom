# ------------------------------------------------------------------------------
# FindEIGEN3.cmake
#
# Compatibility shim for legacy packages expecting EIGEN3-style CMake variables
# and targets. This module ensures interoperability with packages like pcl_ros
# that rely on deprecated conventions.
#
# This file assumes that find_package(Eigen3 REQUIRED) has already been called.
#
# Targets Provided:
#   - EIGEN3::EIGEN3 (ALIAS of Eigen3::Eigen, if it exists)
#
# Variables Exported:
#   - EIGEN3_FOUND: Always TRUE (indicates Eigen3 was found)
#   - EIGEN3_INCLUDE_DIRS: Set from Eigen3_INCLUDE_DIRS
#   - EIGEN3_LIBRARIES: Empty string (Eigen is header-only)
# ------------------------------------------------------------------------------

# Require Eigen3 to be found beforehand
find_package(Eigen3 REQUIRED)

# Define legacy alias target if not already defined
# Some older packages (e.g., pcl_ros) expect EIGEN3::EIGEN3 instead of Eigen3::Eigen
if(NOT TARGET EIGEN3::EIGEN3 AND TARGET Eigen3::Eigen)
  add_library(EIGEN3::EIGEN3 ALIAS Eigen3::Eigen)
endif()

# Export expected legacy variables to mimic classic FindEIGEN3 behavior

# Mark Eigen3 as found
set(EIGEN3_FOUND TRUE)

# Export include directories (may be empty if not defined upstream)
set(EIGEN3_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})

# Eigen3 is header-only; no library to link against
set(EIGEN3_LIBRARIES "")
