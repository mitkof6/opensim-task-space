# TaskSpace
################################################################################
# Source files

set(Target TaskSpace)

file(GLOB INCS *.h internal/*.h)
file(GLOB SRCS *.cpp internal/*.cpp)

################################################################################
# Dependences

include_directories(${CMAKE_SOURCE_DIR}/src/util)
include_directories(${CMAKE_SOURCE_DIR}/src/opensimutil)

################################################################################
# Build library

addLibrary(
  #MASK_UNITY_BUILD
  KIT ${Target}
  LINKLIBS ${OpenSim_LIBRARIES} OpenSimUtil Util
  INCLUDES ${INCS}
  SOURCES ${SRCS}
  )
