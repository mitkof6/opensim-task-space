# Examples
################################################################################
# Dependences

include_directories(${CMAKE_SOURCE_DIR}/src/util)
include_directories(${CMAKE_SOURCE_DIR}/src/opensimutil)
include_directories(${CMAKE_SOURCE_DIR}/src/taskspace)
include_directories(${CMAKE_SOURCE_DIR}/src/taskspaceik)
include_directories(${CMAKE_SOURCE_DIR}/src/taskspacemc)
include_directories(${CMAKE_SOURCE_DIR}/src/muscleoptimization)
include_directories(${CMAKE_SOURCE_DIR}/src/numbersystems)

################################################################################
# Create examples

file(GLOB PROGS
  "AbsoluteCoordinates.cpp"
  "CloseKinematicChain.cpp"
  "TSIKBatch.cpp"
  "ReachingSimulation.cpp"
  "TaskSpaceInverseKinematics.cpp"
  "TaskSpaceComputedMuscleControl.cpp"
  )

# examples can be removed for list
# list(REMOVE_ITEM PROGS ${CMAKE_CURRENT_SOURCE_DIR}/testSetControl.cpp)

addExamples(
  TESTPROGRAMS ${PROGS}
  #DATAFILES ${DATA_FILES}
  LINKLIBS ${OpenSim_LIBRARIES} ${PROJECT_LIBRARIES}
  )
