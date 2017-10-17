################################################################################
# subdirectories

include_directories(includes)

add_subdirectory(util)
add_subdirectory(opensimutil)
add_subdirectory(taskspace)
add_subdirectory(taskspaceik)
add_subdirectory(taskspacemc)
add_subdirectory(muscleoptimization)
add_subdirectory(numbersystems)

if(BUILD_EXAMPLES)
  #enable_testing()
  #include(CTest)
  add_subdirectory(examples)
endif()
