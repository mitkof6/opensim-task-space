#include "TaskBasedController.h"

#include "TaskDynamics.h"
#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

TaskBasedController::TaskBasedController(Model& model, TaskDynamics* taskDynamics)
    : taskDynamics(taskDynamics)
{
    vector<string> dummy;
    addReserveActuators(model, dummy, controlIndicies);
}

TaskBasedController::~TaskBasedController()
{
}

void TaskBasedController::computeControls(const SimTK::State& s,
					  SimTK::Vector& controls) const
{
    // compute torques
    Vector generalizedForces = taskDynamics->calcTaskTorques(s);

    // assign controls to reserve actuators
    SimTK_ASSERT(generalizedForces.size() == controlIndicies.size(),
		 "Size does not agree");
    int j = 0;
    for (int i = 0; i < controlIndicies.size(); i++)
    {
        controls[controlIndicies[i]] += generalizedForces[j];
        j++;
    }
}
