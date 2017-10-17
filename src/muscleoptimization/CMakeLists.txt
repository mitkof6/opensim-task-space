# MuscleOptimization
################################################################################
# Source files

set(Target MuscleOptimization)

file(GLOB INCS *.h internal/*.h)
file(GLOB SRCS *.cpp internal/*.cpp)

################################################################################
# Dependences

include_directories(${CMAKE_SOURCE_DIR}/src/opensimutil)


################################################################################
# Build library

addLibrary(
  #MASK_UNITY_BUILD
  KIT ${Target}
  LINKLIBS ${OpenSim_LIBRARIES} OpenSimUtil
  INCLUDES ${INCS}
  SOURCES ${SRCS}
  )
