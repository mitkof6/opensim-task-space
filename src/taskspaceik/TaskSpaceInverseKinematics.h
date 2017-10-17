#ifndef TASK_SPACE_INVERSE_KINEMATICS_H
#define TASK_SPACE_INVERSE_KINEMATICS_H

#include "internal/TaskSpaceIKDLL.h"

#include <OpenSim/Simulation/Model/Model.h>
#include "ConstraintModel.h"

namespace OpenSim
{
    //class Model;
    class Kinematics;
    class BodyKinematics;
    class ForceReporter;
    class IKTaskManager;
    class TaskBasedForce;

    /**
     * Performs task space inverse kinematics [1].
     *
     * [1] ... TODO
     *
     * @author Dimitar Stanev
     */
    class TaskSpaceIK_API TaskSpaceInverseKinematics
    {
    public:

        TaskSpaceInverseKinematics(ConstraintModel::Type,
                                   std::string modelPath, std::string trcFile,
                                   double startTime, double endTime,
                                   std::string markersPath = "",
                                   bool filter = false, bool verbose = false);

        ~TaskSpaceInverseKinematics();

        void run();

        void printResults(std::string prefix, std::string dir);

    private:

        // private members

        double startTime, endTime;
        bool verbose;
        Model model;
        Kinematics* kinematicsAnalysis;
        BodyKinematics* bodyKinematicsAnalysis;
        Storage stateStorage;
        TaskBasedForce* controller;
        IKTaskManager* taskManager;
    };
}

#endif
