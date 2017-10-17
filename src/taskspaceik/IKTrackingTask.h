#ifndef IK_TRACKING_TASK_H
#define IK_TRACKING_TASK_H

#include "internal/TaskSpaceIKDLL.h"

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/FunctionSet.h>
#include <SimTKcommon/Simmatrix.h>
#include <OpenSim/Common/GCVSplineSet.h>

#include "Quaternion.h"
#include "Spline.h"

#include "tree.h"

namespace OpenSim
{
    /* IKTrackingTask *********************************************************/

    // forward declaration
    class IKTaskData;

    /**
     * An abstract class used for polymorphism of the tracking tasks. The
     * tracking tasks can be on of the following types [positional,
     * orientational, spatial].
     *
     * @author Dimitar Stanev
     */
    class TaskSpaceIK_API IKTrackingTask : public ModelComponent
    {
        OpenSim_DECLARE_ABSTRACT_OBJECT(IKTrackingTask, ModelComponent);
    public:

        IKTrackingTask(const Array<double>& t);

        virtual ~IKTrackingTask();

        /**
         * For a given time instance compute the desired positions, velocities
         * and accelerations.
         *
         * @param t desired time
         * @param x [out] desired position
         * @param v [out] desired velocity
         * @param a [out] desired acceleration
         */
        virtual void calcGoal(double t, SimTK::Vector& x, SimTK::Vector& v,
                              SimTK::Vector& a) = 0;

        /**
         * Pre-process function that evaluates the desired body transformations
         * from measured marker trajectories.
         */
        virtual void calcTransitions(const SimTK::Transform& T0,
                                     tree_node_<IKTaskData*>& current);

    protected:

        SimTK::Vector time;

        std::vector<int> firstDerivative;
        std::vector<int> secondDerivative;
        std::vector<SimTK::Spline_<SimTK::Vec3> > smoothSplines;
    };

    /* IKPositionTrackingTask *************************************************/

    /**
     * Position based tracking task, which tracks the coordinates of a single
     * marker.
     *
     * @author Dimitar Stanev
     */
    class TaskSpaceIK_API IKPositionTrackingTask : public IKTrackingTask
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(IKPositionTrackingTask, IKTrackingTask);
    public:

        /**
         * Constructor
         *
         * @param x, y, z the GSVSpline of the marker coordinate
         */
        IKPositionTrackingTask(const Array<double>& time,
                               Function* markerX, Function* markerY, Function* markerZ);

        ~IKPositionTrackingTask();

        void calcGoal(double t, SimTK::Vector& x, SimTK::Vector& u,
                      SimTK::Vector& a) override;

        void calcTransitions(const SimTK::Transform& T0,
                             tree_node_<IKTaskData*>& current) override;
    };

    /* IKOrientationTrackingTask **********************************************************/

    /**
     * An orientation based tracking task, that tracks more than 3
     * markers and computes the transformation between each frame.
     *
     * @author Dimitar Stanev
     */
    class TaskSpaceIK_API IKOrientationTrackingTask : public IKTrackingTask
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(IKOrientationTrackingTask, IKTrackingTask);
    public:

        IKOrientationTrackingTask(const Array<double>& time,
                                  const GCVSplineSet& markerSplineSet, const std::vector<int>& indicies);

        ~IKOrientationTrackingTask();

        virtual void calcGoal(double t, SimTK::Vector& x, SimTK::Vector& v,
                              SimTK::Vector& a) override;

        virtual void calcTransitions(const SimTK::Transform& T0,
                                     tree_node_<IKTaskData*>& current) override;

    protected:

        void calcTransformation(const SimTK::Transform& T0,
                                const std::vector<SimTK::Transform>& transitions,
                                Quaternion<double>* orientations, SimTK::Vector_<SimTK::Vec3>& positions);

    protected:

        // private variables

        Spline<Quaternion<double>, double>* spline;
    };

    /* IKSpatialTrackingTask **************************************************************/

    /**
     * A spatial based tracking task (position and orientation), that tracks more than 3
     * markers and computes the transformation between each frame.
     *
     * @author Dimitar Stanev
     */
    class TaskSpaceIK_API IKSpatialTrackingTask : public IKOrientationTrackingTask
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(IKSpatialTrackingTask, IKOrientationTrackingTask);
    public:

        IKSpatialTrackingTask(const Array<double>& time,
                              const GCVSplineSet& markerSplineSet, const std::vector<int>& indicies);

        ~IKSpatialTrackingTask();

        void calcGoal(double t, SimTK::Vector& x, SimTK::Vector& v,
                      SimTK::Vector& a) override;

        void calcTransitions(const SimTK::Transform& T0,
                             tree_node_<IKTaskData*>& current) override;
    };
}

#endif
