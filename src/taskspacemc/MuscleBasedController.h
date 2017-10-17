#ifndef MUSCLE_BASED_CONTROLLER_H
#define MUSCLE_BASED_CONTROLLER_H

#include "internal/TaskSpaceMCDLL.h"

#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Common/Storage.h>

namespace OpenSim
{
    class TaskDynamics;
    class MuscleOptimization;

    /**
     * Computes the generalized forces following a task oriented controller
     * approach and applies it to the CoordinateActuators.
     *
     * The controller appends a set of CooridnateActuators that are used as
     * torque drivers. We choose not to use OpenSim::Force for this purpose
     * because we would like to disable the controller without the need to
     * invalidate the state, as is the case with a Force component.
     *
     * @author Dimitar Stanev
     */
    class TaskSpaceMC_API MuscleBasedController : public Controller
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(MuscleBasedController, Controller);
    public:

        /**
         * The controller does not takes ownership of pointer objects.
         */
        MuscleBasedController(Model& model, TaskDynamics* taskDynamics,
                              MuscleOptimization* muscleOptimization);

        ~MuscleBasedController();

        void printResults(std::string prefix, std::string dir);

    protected:

        // Controller methods

        void computeControls(const SimTK::State& s, SimTK::Vector& controls)
            const override;

        // ModelComponent methods

        void extendInitStateFromProperties(SimTK::State& s) const override;

    private:

        SimTK::ReferencePtr<TaskDynamics> taskDynamics;
        SimTK::ReferencePtr<MuscleOptimization> muscleOptimization;
        mutable Storage controlStorage;
        mutable SimTK::Vector excitations;
    }; // end of class
}  // end of namespace OpenSim

#endif
