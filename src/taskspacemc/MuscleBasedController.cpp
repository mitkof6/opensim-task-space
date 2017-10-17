#include "MuscleBasedController.h"

#include "TaskDynamics.h"
#include "MuscleOptimization.h"
#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

MuscleBasedController::MuscleBasedController(Model& model, TaskDynamics* taskDynamics,
                                             MuscleOptimization* muscleOptimization)
    : taskDynamics(taskDynamics), muscleOptimization(muscleOptimization),
      controlStorage(controlStorage)

{
    vector<string> dummy1;
    vector<int> dummy2;
    addReserveActuators(model, dummy1, dummy2);
}

MuscleBasedController::~MuscleBasedController()
{
}

void MuscleBasedController::printResults(string prefix, string dir)
{
    controlStorage.print(dir + "/" + prefix + "_Controls.sto");
    taskDynamics->printResults(prefix, dir);
}

void MuscleBasedController::computeControls(const SimTK::State& s,
                                            SimTK::Vector& controls) const
{
    // compute torques
    Vector generalizedForces = taskDynamics->calcTaskTorques(s);

    // compute muscle activations
    muscleOptimization->tryToOptimize(s, generalizedForces, excitations);

    // update controls
    controls += excitations;

    controlStorage.append(s.getTime(), controls.size(), &controls[0]);
}

void MuscleBasedController::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    excitations = Vector(_model->getActuators().getSize(), 0.0);

    // setup activation storage
    Array<string> labels;
    labels.append("time");
    const Set<Actuator>& act = _model->getActuators();
    for (int i = 0; i < act.getSize(); i++)
    {
        labels.append(act[i].getName());
    }
    controlStorage.setColumnLabels(labels);
    controlStorage.reset(0);
}
