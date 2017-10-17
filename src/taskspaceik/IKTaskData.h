#ifndef IK_TASK_DATA_H
#define IK_TASK_DATA_H

#include "internal/TaskSpaceIKDLL.h"

#include <OpenSim/Simulation/Model/ModelComponent.h>

#include <OpenSim/Simulation/Model/Marker.h>

namespace OpenSim
{
    class IKTrackingTask;
    class TaskPrimitive;

    /**
    * Helper class that keeps useful information about the task, the
    * corresponding markers, the tracking tasks and it is used to improve the
    * time to find these quantities during simulation.
    *
    * @author Dimitar Stanev
    */
    class TaskSpaceIK_API IKTaskData : public ModelComponent
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(IKTaskData, ModelComponent);
    public:

        /**
        * Helper data structure for marker - and gcvspline index
        */
        struct MarkerInfo
        {
            MarkerInfo()
            {
            }

            MarkerInfo(int gcvIndex, OpenSim::Marker* m)
            {
                coordinateIndex = gcvIndex;
                marker = m;
            }

            /*
            * coordinateIndex: is the correspondence of the marker
            * in the MarkerData
            */
            int coordinateIndex;
            Marker* marker; // it is a reference to the marker owned by model
        };

        IKTaskData(std::string name);

        ~IKTaskData();

        std::string getName() const;

        /**
        * Get non-constant reference to the underlying task.
        */
        TaskPrimitive* updTask();

        /**
        * Takes ownership of the task object.
        */
        void setTask(TaskPrimitive* task);

        double getKp() const;
        void setKp(double kp);

        double getKv() const;
        void setKv(double kv);

        void addMarkerInfo(MarkerInfo mInfo);
        std::vector<MarkerInfo>& updMarkerInfo();

        IKTrackingTask* updTrackingTask();
        void setTrackingTask(IKTrackingTask* trackingTask);

        const std::vector<SimTK::Transform>& getTransitions();
        void setTransitions(const std::vector<SimTK::Transform>& tranistions);

        std::vector<int> getCoordinateInicies();

    protected:

        // ModelComponent methods

        void extendConnectToModel(Model& model) override;

    protected:

        // data members

        /**
        * The name of the task, it is used to locate a task in the priority
        * tree.
        */
        std::string name;

        TaskPrimitive* task;

        /**
        * Tracking gains for the specific task.
        */
        double kp, kv;

        std::vector<MarkerInfo> markers;
        IKTrackingTask* trackingTask;
        std::vector<SimTK::Transform> transitions;
    };
}

#endif
