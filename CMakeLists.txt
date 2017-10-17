CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

################################################################################
# project

project(OpenSimEngine VERSION 1.0.0)

# modules
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(cmake/CMakeHelpers.cmake)

# build options
option(USE_UNITY_BUILD "For fast building" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_DOCUMENTATION
  "Create and install the HTML based API documentation (requires Doxygen)" OFF)

# any library is appended here
set(PROJECT_LIBRARIES "" CACHE INTERNAL "Contains all project libraries")

# configure project global settings that are include in the source
set(BASE_DIR "${PROJECT_SOURCE_DIR}")
set(INI_PATH "${PROJECT_SOURCE_DIR}/data/setup.ini")
if(${CMAKE_HOST_SYSTEM_NAME} MATCHES Windows)
  set(PAUSE "true" CACHE STRING
    "Set true to pause main after completion, else false (for ctest)")
else()
  set(PAUSE "false" CACHE STRING
    "Set true to pause main after completion, else false (for ctest)")
endif()
set(CMAKE_EXTRA_INCLUDE_FILES src/includes/Settings.h)
configure_file(
  "${PROJECT_SOURCE_DIR}/src/includes/Settings.h.in"
  "${PROJECT_SOURCE_DIR}/src/includes/Settings.h")
set(CMAKE_EXTRA_INCLUDE_FILES)

# install layout
# Layout. This works for all platforms:
#   * <prefix>/lib/cmake/<PROJECT-NAME>
#   * <prefix>/lib/
#   * <prefix>/include/
#   * <prefix>/bin/
#   * <prefix>/doc/
#   * <prefix>/data/
set(CMAKE_INSTALL_BINDIR bin)
set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_DOCDIR ${CMAKE_INSTALL_PREFIX}/docs)
set(CMAKE_INSTALL_CONFIG ${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME})
set(CMAKE_INSTALL_DATADIR ${CMAKE_INSTALL_PREFIX}/data)

# build executables and libraries in the same place
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR} CACHE INTERNAL "")
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR} CACHE INTERNAL "")

# define library postfix so that different builds will produce
# distinguished libraries
set(CMAKE_RELEASE_POSTFIX "_r" CACHE string "Release postfix")
set(CMAKE_DEBUG_POSTFIX "_d" CACHE string "Debug postfix")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE string
  "Release with debug info postfix")
set(CMAKE_MINSIZEREL_POSTFIX "_mr" CACHE string
  "Minimum size release postfix")
mark_as_advanced(CMAKE_RELEASE_POSTFIX)
mark_as_advanced(CMAKE_DEBUG_POSTFIX)
mark_as_advanced(CMAKE_RELWITHDEBINFO_POSTFIX)
mark_as_advanced(CMAKE_MINSIZEREL_POSTFIX)

# for grouping into folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# c++11
if(${CMAKE_CXX_COMPILER_ID} MATCHES GNU OR ${CMAKE_CXX_COMPILER_ID} MATCHES Clang)
  # Using C++11 on OSX requires using libc++ instead of libstd++.
  # libc++ is an implementation of the C++ standard library for OSX.
  if(APPLE)
    if(XCODE)
      set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
      set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
    endif()
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  endif()
endif()

# In addition to the platform name we need to know the Application Binary
# Interface (ABI) we're building for. Currently that is either x86, meaning
# 32 bit Intel instruction set, or x64 for 64 bit Intel instruction set.
if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
  set(PLATFORM_ABI x64)
else()
  set(PLATFORM_ABI x86)
endif()
set(BUILD_PLATFORM "${CMAKE_HOST_SYSTEM_NAME}:${PLATFORM_ABI}" CACHE STRING
  "This is the platform and ABI we're building for."  FORCE)

# remove annoying MSVC warnings
if(MSVC)
  add_definitions(
    "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4996 /wd4800 /wd4267 /wd4244 /wd4018")
  # double to float truncation, conversation from double to float,
  # overflow in floating point
  add_definitions(
    "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4305 /wd4838 /wd4056 /wd4068")
endif()

################################################################################
# find libraries

find_package(Simbody
  REQUIRED
  HINTS "${DEPENDENCIES_DIR}/simbody/cmake")
if(Simbody_FOUND)
  message(STATUS "Simbody found at ${Simbody_DIR}")
  include_directories(${Simbody_INCLUDE_DIR})
  link_directories(${Simbody_LIB_DIR})
else()
  message(FATAL_ERROR "Can't find Simbody at ${Simbody_DIR}")
endif()

find_package(OpenSim
  REQUIRED
  HINTS "${DEPENDENCIES_DIR}/opensim-core/cmake")
if(OpenSim_FOUND)
  message(STATUS "OpenSim found at ${OpenSim_DIR}")
  include_directories(${OpenSim_INCLUDE_DIRS})
  link_directories(${OpenSim_LIB_DIR})
else()
  message(FATAL_ERROR "Can't find OpenSim at ${OpenSim_DIR}")
endif()

################################################################################
# subdirectories

add_subdirectory(src)
add_subdirectory(doc)

################################################################################
# installation (https://github.com/forexample/package-example)

# include module with function 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

# configuration
set(TARGET_EXPORT_NAME "${PROJECT_NAME}-targets" CACHE 
	INTERNAL "Internal variable that is referenced by library generation")
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(VERSION_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}Config.cmake")

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Note: PROJECT_VERSION is used as a VERSION
write_basic_package_version_file(
  "${VERSION_CONFIG}" COMPATIBILITY SameMajorVersion
  )

# Configure '<PROJECT-NAME>Config.cmake'
configure_package_config_file(
  "cmake/Config.cmake.in"
  "${PROJECT_CONFIG}"
  INSTALL_DESTINATION "${CMAKE_INSTALL_CONFIG}"
  )

# intall configuration
#   * <prefix>/lib/cmake/Foo/FooConfig.cmake
#   * <prefix>/lib/cmake/Foo/FooConfigVersion.cmake
install(
  FILES "${PROJECT_CONFIG}" "${VERSION_CONFIG}"
  DESTINATION "${CMAKE_INSTALL_CONFIG}"
  )

# install target configuration
#   * <prefix>/lib/cmake/Foo/FooTargets.cmake
install(
	EXPORT ${TARGET_EXPORT_NAME}
	DESTINATION "${CMAKE_INSTALL_CONFIG}"
)

# data directory
install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/data/
  DESTINATION "${CMAKE_INSTALL_DATADIR}"
  )
# others dirs can be copied to the install dir

################################################################################
# CPack
# not working properly yet

set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
set(CPACK_RPM_PACKAGE_REQUIRES "")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "")
set(CPACK_PACKAGE_VENDOR "Dimitar Stanev")
set(CPACK_PACKAGE_CONTACT ${CPACK_PACKAGE_VENDOR})
set(CPACK_PACKAGE_DESCRIPTION_FILE "${PROJECT_SOURCE_DIR}/readme.org")
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE.txt")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_DEBIAN_PACKAGE_SECTION "Development")
set(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
set(CPACK_RPM_PACKAGE_LICENSE "LICENSE.txt")
set(CPACK_STRIP_FILES TRUE)
include(CPack)
