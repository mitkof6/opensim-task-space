# Util
#################################################################################################
# Source files

set(Target Util)

file(GLOB INCS *.h internal/*.h)
file(GLOB SRCS *.cpp internal/*.cpp)

#################################################################################################
# Dependences

#################################################################################################
# Build library

addLibrary(
    KIT ${Target}
    LINKLIBS ${OpenSim_LIBRARIES}
    INCLUDES ${INCS}
    SOURCES ${SRCS}
)