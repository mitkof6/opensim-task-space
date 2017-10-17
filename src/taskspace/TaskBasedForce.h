#ifndef TASK_BASED_FORCE_H
#define TASK_BASED_FORCE_H

#include "internal/TaskSpaceDLL.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Force.h>

namespace OpenSim {
    class TaskDynamics;

    /**
     * Computes the generalized forces following a task oriented
     * controller approach.
     *
     * @author Dimitar Stanev
     */
    class TaskSpace_API TaskBasedForce : public Force {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskBasedForce, Force);
    public:

        /**
         * The controller does not takes ownership of the TaskDynamics object,
         * and the user should manually add this component to the model.
         */
        TaskBasedForce(TaskDynamics* taskDynamics);

        virtual ~TaskBasedForce();

        void printResults(std::string prefix, std::string dir);

    protected:

        // Force methods

        void computeForce(const SimTK::State& state,
                          SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                          SimTK::Vector& generalizedForces) const override;

        void extendInitStateFromProperties(SimTK::State& s) const override;

    protected:

        TaskDynamics* taskDynamics;
        mutable Storage forcesSto;
    }; // end of class
}  // end of namespace OpenSim

#endif
