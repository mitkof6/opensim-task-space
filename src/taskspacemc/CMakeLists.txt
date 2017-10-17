# TaskSpaceMC
################################################################################
# Source files

set(Target TaskSpaceMC)

file(GLOB INCS *.h internal/*.h)
file(GLOB SRCS *.cpp internal/*.cpp)

################################################################################
# Dependences

include_directories(${CMAKE_SOURCE_DIR}/src/opensimutil)
include_directories(${CMAKE_SOURCE_DIR}/src/taskspace)
include_directories(${CMAKE_SOURCE_DIR}/src/taskspaceik) #muscle be removed later
include_directories(${CMAKE_SOURCE_DIR}/src/muscleoptimization)

################################################################################
# Build library

addLibrary(
  #MASK_UNITY_BUILD
  KIT ${Target}
  LINKLIBS ${OpenSim_LIBRARIES} TaskSpace TaskSpaceIK MuscleOptimization OpenSimUtil
  INCLUDES ${INCS}
  SOURCES ${SRCS}
  )
