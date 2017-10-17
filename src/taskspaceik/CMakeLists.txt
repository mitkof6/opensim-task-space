# TaskSpaceIK
################################################################################
# Source files

set(Target TaskSpaceIK)

file(GLOB INCS *.h internal/*.h)
file(GLOB SRCS *.cpp internal/*.cpp)

################################################################################
# Dependences

include_directories(${CMAKE_SOURCE_DIR}/src/opensimutil)
include_directories(${CMAKE_SOURCE_DIR}/src/taskspace)
include_directories(${CMAKE_SOURCE_DIR}/src/numbersystems)

################################################################################
# Build library

addLibrary(
  #MASK_UNITY_BUILD
  KIT ${Target}
  LINKLIBS ${OpenSim_LIBRARIES} NumberSystems TaskSpace OpenSimUtil
  INCLUDES ${INCS}
  SOURCES ${SRCS}
  )
