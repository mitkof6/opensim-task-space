#ifndef IK_TASK_MANAGER_H
#define IK_TASK_MANAGER_H

#include "internal/TaskSpaceIKDLL.h"

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Common/MarkerData.h>
#include "ConstraintModel.h"
#include "tree.h"

namespace OpenSim {

    class TaskDynamics;
    class TaskPrimitive;
    class IKTaskData;

    /**
     * The task manager is responsible to create and group the tasks and the
     * tracking tasks based on the model and the provided marker set. Also during
     * task space inverse kinematics this class communicates and updates the
     * tracking goal of each task and applies the computed torque to drive the
     * mode.
     *
     * A task is the representation of a goal by a Jacobian, while a tracking
     * task is the reconstructed goal from measurements. The task's acceleration
     * tracks the tracking task's goal.
     *
     * @author Dimitar Stanev
     */
    class TaskSpaceIK_API IKTaskManager : public ModelComponent {
        OpenSim_DECLARE_CONCRETE_OBJECT(IKTaskManager, ModelComponent);
    public:

        /**
         * Pre-initialized in the constructor, and should not be changed,
         * because their effect is not accounted currently
         */
        OpenSim_DECLARE_PROPERTY(linearResampling, double,
                                 "Re-sampling of .trc file with a constant interval (typically 0.01)");
        OpenSim_DECLARE_PROPERTY(cutOffFrequencty, double,
                                 "The .trc coordinates are filtered by low pass filter (typically 6Hz)");
        OpenSim_DECLARE_PROPERTY(iirOrder, int, "The IIR filter order (typically 50)");
        OpenSim_DECLARE_PROPERTY(splineOrder, int, "The smooth spline order (typically 5)");

        IKTaskManager(ConstraintModel::Type type, double t0,
                      const MarkerData& markerData, bool filter = false,
                      bool verbose = false);

        ~IKTaskManager();

        /**
         * Should be called to initialize some internal variables after the model
         * is created and build. This method must be initiated before inverse
         * kinematics run() method.
         */
        void initializeFromState(SimTK::State& s);

        /**
         * For a given desired time (t + T), the desired goal of the tasks is
         * updated.
         *
         * @param s the current state at time t @param targetTime the target time
         * t + T
         */
        void updateGoals(const SimTK::State& s, double targetTime) const;

        /**
         * Prints the corresponding internal storage.
         */
        void printResults(std::string prefix, std::string dir);

        /**
         * Returns the marker data from the recorded motion.
         */
        MarkerData& getMarkerData();

        /**
         * Returns the time vector that corresponds to the recorded motion.
         */
        Array<double> getTimeVector();

        TaskDynamics& updTaskDynamics();

    protected:

        // ModelComponent methods

        void extendConnectToModel(Model& model) override;

    private:

        /**
         * This method is called by extendConnectToModel in order to access some
         * structural data for the creation of the tasks.
         */
        void constructTasksFromMarkers();

        /**
         * This method is called by extendConnectToModel in order to access some
         * structural data for the creation of the tracking tasks.
         */
        void constructTrackingTasks();

        /**
         * Helper function used to append a set of labels for tracking the
         * corresponding task.
         */
        void appendLabelsForTask(Array<std::string>& labels,
				 const TaskPrimitive* task);

        /**
         * Helper function that populates the task goals into the array that will
         * be inserted into the desired command storage.
         */
        void populateDesiredCommand(Array<double>& goalData,
                                    const SimTK::Vector& xd,
				    const SimTK::Vector& vd,
				    const SimTK::Vector& ad,
                                    const TaskPrimitive* task) const;

        /**
         * Calculates marker error, min, max, total RMS. Then appends the marker
         * error internal storage and finally it returns the total RMS.
         */
        double calcMarkerError(const SimTK::State& s) const;

        /**
         * Given a AbstractTaskData tree, find the corresponding task data for a
         * given name.
         */
        tree<OpenSim::IKTaskData*>::iterator find(const std::string& name);

        /**
         * Given a AbstractTaskData tree, prints the hierarchy.
         */
        void printTree();

    private:

        // private members

        bool verbose;
        mutable double maxRMS;
        double startTime;
        TaskDynamics* taskDynamics;
        MarkerData markerData;
        Storage markerDataStorage;
        GCVSplineSet markerSplineSet;
        Array<std::string> markerNames;
	tree<IKTaskData*> bodyTree;
        mutable OpenSim::Storage desiredCommands, markerError;
    };
}

#endif
