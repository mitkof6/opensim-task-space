#ifndef TASK_SPACE_COMPUTED_MUSCLE_CONTROL_H
#define TASK_SPACE_COMPUTED_MUSCLE_CONTROL_H

#include "internal/TaskSpaceMCDLL.h"

#include <OpenSim/Simulation/Model/Model.h>
#include "ConstraintModel.h"

namespace OpenSim
{

    /* TaskSpaceComputedMuscleControl ****************************************/

    // forward declaration
    class Kinematics;
    class BodyKinematics;
    class ForceReporter;
    class ModelAdapter;
    class IKTaskManager;

    /**
     * Task space computed muscle control.
     *
     * @author Dimitar Stanev
     */
    class TaskSpaceMC_API TaskSpaceComputedMuscleControl {
    public:

        TaskSpaceComputedMuscleControl(ConstraintModel::Type type,
                                       std::string modelPath, std::string trcFile,
				       double startTime, double endTime,
                                       std::string externalLoadsXML = "",
				       std::string externalLoadsData = "",
                                       std::string externalLoadsMot = "",
                                       std::string markersPath = "",
                                       bool filter = false, bool verbose = false);

        ~TaskSpaceComputedMuscleControl();

        void run();

        void printResults(std::string prefix, std::string dir);

    private:

        // private members

        double startTime, endTime;
        bool verbose;
        Model model;
        Kinematics* kinematicsAnalysis;
        BodyKinematics* bodyKinematicsAnalysis;
        ForceReporter* forceReporter;
        Storage stateStorage, controlsStorage;
        IKTaskManager* taskManager;
        SimTK::ReferencePtr<ModelAdapter> modelAdapter;
        SimTK::ReferencePtr<Storage> externalForceStorage;
    };
}

#endif
