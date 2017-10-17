# NumberSystems
################################################################################
# Source files

set(Target NumberSystems)

file(GLOB INCS *.h internal/*.h)
file(GLOB SRCS *.cpp internal/*.cpp)

################################################################################
# Dependences

################################################################################
# Build library

addLibrary(
  KIT ${Target}
  INCLUDES ${INCS}
  SOURCES ${SRCS}
  )

# number systems warnings
if(${CMAKE_CXX_COMPILER_ID} MATCHES GNU OR ${CMAKE_CXX_COMPILER_ID} MATCHES Clang)
  set_target_properties(${Target}
    PROPERTIES COMPILE_FLAGS "-fpermissive -Wno-narrowing")
endif()
