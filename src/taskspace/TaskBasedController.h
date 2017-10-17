#ifndef TASK_BASED_CONTROLLER_H
#define TASK_BASED_CONTROLLER_H

#include "internal/TaskSpaceDLL.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Control/Controller.h>

namespace OpenSim
{
    class TaskDynamics;

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
    class TaskSpace_API TaskBasedController : public Controller
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskBasedController, Controller);
    public:

        /**
         * The controller does not takes ownership of the TaskDynamics object,
         * and the user should manually add this component to the model.
         */
        TaskBasedController(Model& model, TaskDynamics* taskDynamics);

        virtual ~TaskBasedController();

    protected:

        // Controller methods

        virtual void computeControls(const SimTK::State& s, SimTK::Vector& controls)
            const override;

    protected:

        TaskDynamics* taskDynamics;

        std::vector<int> controlIndicies;
    }; // end of class
}  // end of namespace OpenSim

#endif
