# OpenSimUtil
################################################################################
# Source files

set(Target OpenSimUtil)

file(GLOB INCS *.h internal/*.h)
file(GLOB SRCS *.cpp internal/*.cpp)

################################################################################
# Dependences

include_directories(${CMAKE_SOURCE_DIR}/src/util)

################################################################################
# Build library

addLibrary(
  KIT ${Target}
  LINKLIBS ${OpenSim_LIBRARIES} Util
  INCLUDES ${INCS}
  SOURCES ${SRCS}
  )
