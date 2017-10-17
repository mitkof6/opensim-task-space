#include "TaskSpaceInverseKinematics.h"

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Analyses/ForceReporter.h>

#include "IKTaskManager.h"
#include "TaskDynamics.h"
#include "TaskBasedForce.h"
#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace std;

class CustomTaskBasedForce : public TaskBasedForce {
    OpenSim_DECLARE_CONCRETE_OBJECT(CustomTaskBasedForce, TaskBasedForce);
public:

    CustomTaskBasedForce(IKTaskManager* taskManager)
        : TaskBasedForce(&taskManager->updTaskDynamics()),
          taskManager(taskManager) {
    }

    void computeForce(const SimTK::State& s,
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                      SimTK::Vector& generalizedForces) const override {
        taskManager->updateGoals(s, s.getTime());

        Super::computeForce(s, bodyForces, generalizedForces);
    }

private:

    IKTaskManager* taskManager;
};

TaskSpaceInverseKinematics::TaskSpaceInverseKinematics(
    ConstraintModel::Type type,
    string modelPath, string trcFile, double t0, double tf, string markersPath,
    bool filter, bool v)
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

    // add TaskManager
    taskManager = new IKTaskManager(type, startTime, markerData, filter, verbose);
    model.addModelComponent(taskManager);

    // add force controller
    controller = new CustomTaskBasedForce(taskManager);
    model.addForce(controller);

    SimTK::State& s = model.initSystem();
    setUseMuscles(s, model, false);
    s.setTime(startTime); // important

    // initialize system
    calcInitialState(s, model, taskManager->getMarkerData());
    taskManager->initializeFromState(s);
}

TaskSpaceInverseKinematics::~TaskSpaceInverseKinematics() {
}

void TaskSpaceInverseKinematics::run() {
    SimTK::State& s = model.updWorkingState();

    // setup integrator
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());// good error 22s
    //SimTK::RungeKuttaFeldbergIntegrator integrator(model.getMultibodySystem());// good error 27s
    //SimTK::SemiExplicitEulerIntegrator integrator(model.getMultibodySystem(), 0.01);// good error with 0.01 12s
    //SimTK::SemiExplicitEuler2Integrator integrator(model.getMultibodySystem());// 22s
    //SimTK::VerletIntegrator integrator(model.getMultibodySystem(), 0.01);// good error with 0.01 14s
    //SimTK::ExplicitEulerIntegrator integrator(model.getMultibodySystem(), 0.01);// very good error 20s
    //SimTK::CPodesIntegrator integrator(model.getMultibodySystem(), CPodes::Adams,
    //  CPodes::NonlinearSystemIterationType::Newton); // very bad 222s
    //integrator.setAccuracy(1e-4);

    // integrate
    simulate(s, model, integrator, startTime, endTime, stateStorage);
}

void TaskSpaceInverseKinematics::printResults(string prefix, string dir) {
    kinematicsAnalysis->printResults(prefix, dir);
    bodyKinematicsAnalysis->printResults(prefix, dir);
    stateStorage.print(dir + "/" + prefix + "_State.sto");
    taskManager->printResults(prefix, dir);
    controller->printResults(prefix, dir);
}
