#include "TaskSpaceComputedMuscleControl.h"

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Analyses/ForceReporter.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/Model/ExternalLoads.h>

#include "MuscleOptimization.h"
#include "MuscleOptimizationTarget.h"

#include "IKTaskManager.h"
#include "TaskDynamics.h"
#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

/**
 * Use only for testing, and when not external forces are applied
 */
#define USE_FAST_SIMULATION 0
#define TOMC_UPDATE_TIME 0.01

/* MuscleController ***********************************************************/

class MuscleController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(MuscleController, Controller);
public:

    /**
     * MuscleController does not take ownership of takes manager and muscle
     * optimizer.
     */
    MuscleController(IKTaskManager* taskManager,
                     MuscleOptimization* muscleOptimizer,
                     Storage* controlsStorage);

protected:

    // Controller methods

    void computeControls(const SimTK::State& s, SimTK::Vector& controls)
        const override;

    // ModelComponent methods

    void extendInitStateFromProperties(SimTK::State& s) const override;

private:

    IKTaskManager* taskManager;
    MuscleOptimization* muscleOptimizer;
    SimTK::ReferencePtr<Storage> controlsStorage;
    mutable double updateTime;
    mutable SimTK::Vector activations;
};

/* TaskSpaceComputedMuscleControl *********************************************/

TaskSpaceComputedMuscleControl::TaskSpaceComputedMuscleControl(
    ConstraintModel::Type type,
    string modelPath, string trcFile, double t0, double tf,
    string externalLoadsXML, string externalLoadsData, string externalLoadsMot,
    string markersPath, bool filter, bool v)
    : startTime(t0), endTime(tf), verbose(v) {
    // correct start and end time
    MarkerData markerData = MarkerData(trcFile);
    double tData0 = markerData.getStartFrameTime();
    double tDataF = markerData.getLastFrameTime();
    if (startTime < tData0 || startTime >= endTime) {
        startTime = tData0;
        if (verbose) {
            cout << "Start time corrected to: " << startTime << endl;
        }
    }
    if (endTime > tDataF || endTime <= startTime) {
        endTime = tDataF;
        if (verbose) {
            cout << "End time corrected to: " << endTime << endl;
        }
    }

    model = Model(modelPath);

    // add external loads
    if (SimTK::Pathname::fileExists(externalLoadsXML)) {
        ExternalLoads* externalLoads = new ExternalLoads(model, externalLoadsXML);

        if (SimTK::Pathname::fileExists(externalLoadsData)) {
            string temp, name;
            bool dummy;
            SimTK::Pathname::deconstructPathname(
		externalLoadsData, dummy, temp, name, temp);
            //externalLoads->setDataFileName(externalLoadsData);
            externalForceStorage = new Storage(externalLoadsData);
            externalForceStorage->setName(name);
        }
        if (SimTK::Pathname::fileExists(externalLoadsMot)) {
            externalLoads->setExternalLoadsModelKinematicsFileName(externalLoadsMot);
        }
        externalLoads->setMemoryOwner(false);

        for (int i = 0; i < externalLoads->getSize(); i++) {
            if (externalLoadsData != "") {
                (*externalLoads)[i].setDataSource(*externalForceStorage);
            }
            model.addForce(&(*externalLoads)[i]);
        }
    }

    // build is called prior in order to find markers
    // otherwise there is an internal error during building
    model.buildSystem();
    if (SimTK::Pathname::fileExists(markersPath)) {
        model.append_MarkerSet(MarkerSet(model, markersPath));
    }

    // add analyses
    kinematicsAnalysis = new Kinematics(&model);
    model.addAnalysis(kinematicsAnalysis);

    bodyKinematicsAnalysis = new BodyKinematics(&model);
    model.addAnalysis(bodyKinematicsAnalysis);

    forceReporter = new ForceReporter(&model);
    model.addAnalysis(forceReporter);

    // add TaskManager
    taskManager = new IKTaskManager(type, startTime, markerData, filter, verbose);
    model.addModelComponent(taskManager);

    // muscle optimizer
    auto target = new ApproximationOptimizationTarget(verbose);
    target->setRresidualsPenaly(100);
    SimTK::ReferencePtr<MuscleOptimization> muscleOptimizer;
    muscleOptimizer = new MuscleOptimization(&model, target, verbose);

    // add muscle controller
    MuscleController* muscleController = new MuscleController(taskManager,
							      muscleOptimizer,
                                                              &controlsStorage);
    model.addController(muscleController);

    // initialize
    model.buildSystem();
    State& s = model.initializeState();
    s.setTime(startTime); // important

    // initialize system
    calcInitialState(s, model, taskManager->getMarkerData());
    taskManager->initializeFromState(s);
    muscleOptimizer->initializeFromState(s);
    model.equilibrateMuscles(s);

    // TODO compute initial activations in order to reduce the initial tracking
    // error

    // setup activation storage
    Array<string> labels;
    labels.append("time");
    const Set<Actuator>& act = model.getActuators();
    for (int i = 0; i < act.getSize(); i++) {
        labels.append(act[i].getName());
    }
    controlsStorage.setColumnLabels(labels);
    controlsStorage.reset(0);
}

TaskSpaceComputedMuscleControl::~TaskSpaceComputedMuscleControl() {
}

void TaskSpaceComputedMuscleControl::run() {
    State& s = model.updWorkingState();

    // setup integrator
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    //integrator.setAccuracy(1e-10);

    // integrate
    simulate(s, model, integrator, startTime, endTime, stateStorage);
}

void TaskSpaceComputedMuscleControl::printResults(string prefix, string dir) {
    kinematicsAnalysis->printResults(prefix, dir);
    bodyKinematicsAnalysis->printResults(prefix, dir);
    forceReporter->printResults(prefix, dir);
    stateStorage.print(dir + "/" + prefix + "_State.sto");
    controlsStorage.print(dir + "/" + prefix + "_Controls.sto");

    taskManager->printResults(prefix, dir);
}

/* MuscleController ***********************************************************/

MuscleController::MuscleController(IKTaskManager* tm, MuscleOptimization* mo,
                                   Storage* c)
    : taskManager(tm), muscleOptimizer(mo), updateTime(0), controlsStorage(c) {
}

void MuscleController::computeControls(const State& s, Vector& controls) const {
#if USE_FAST_SIMULATION == 1
    if (abs(s.getTime() - updateTime) < TOMC_UPDATE_TIME) {
        controls += activations;
        return;
    }
#endif

    // update tracking goals
    taskManager->updateGoals(s, s.getTime());

    // compute generalized forces
    Vector generalizedForces =
        taskManager->updTaskDynamics().calcTaskTorques(s);

    // compute muscle activations
    muscleOptimizer->tryToOptimize(s, generalizedForces, activations);

    // update controls
    controls += activations;
    updateTime = s.getTime();

    controlsStorage->append(s.getTime(), controls.size(), &controls[0], true);
}

void MuscleController::extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);

    activations = Vector(_model->getActuators().getSize(), 0.0);
}
