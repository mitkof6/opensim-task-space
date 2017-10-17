#ifndef TASK_PRIMITIVE_H
#define TASK_PRIMITIVE_H

#include "internal/TaskSpaceDLL.h"

#include "AbstractPrimitive.h"
#include "TaskTrackingController.h"

namespace OpenSim
{
    /* TaskPrimitive *********************************************************/

    /**
    * A generic task, that can be used by a task dynamic controller
    * to derive the torques that can achieve the task goal. The
    * task does not use any control law (open loop), so the user
    * is responsible for coordinating the goal.
    *
    * @author Dimitar Stanev
    */
    class TaskSpace_API TaskPrimitive : public AbstractPrimitive
    {
        OpenSim_DECLARE_ABSTRACT_OBJECT(TaskPrimitive, AbstractPrimitive);
    public:

        enum TaskType
        {
            POSITION = 0,
            ORIENTATION,
            SPATIAL,
            UNKNOWN
        };

        TaskPrimitive(std::string body, SimTK::Vec3 bodyOffset,
            TaskTrackingController* controller = new TaskTrackingController());

        virtual ~TaskPrimitive();

        TaskPrimitive* getAsTaskPrimitive() override
        {
            return this;
        }

        /**
        * Get task type
        */
        TaskType getTaskType() const;
        std::string getFromTaskType(TaskType type) const;
        std::string getTaskTypeAsString() const;

        std::string getBody() const;

        SimTK::Vec3 getOffset() const;

        /**
        * Task position
        *
        * \f$ x \in R^t \f$
        */
        virtual SimTK::Vector x(const SimTK::State& s) const = 0;

        /**
        * Task velocity
        *
        * \f$ \dot{x} \in R^t \f$
        */
        virtual SimTK::Vector v(const SimTK::State& s) const = 0;

        /**
        * Task acceleration
        *
        * \f$ \ddot{x} \in R^t \f$
        */
        virtual SimTK::Vector a(const SimTK::State& s) const = 0;

        /**
        * The setGoal can be override, in order to implement a control law,
        * by the subclasses of the TaskPrimitive class.
        */
        void setGoal(const SimTK::State& s, const SimTK::Vector& goal) override;

        /**
        * Inverse dynamic of the current task for a given goal $\f ddot{x} \f$ and the
        * contribution of higher priority tasks $\f \tau_{p*} \f$ and forces.
        *
        * $\f  \tau_k = J^T_p (L_p (\ddot{x} + b) + \bar{J}^T_p \tau_p) \f$
        */
        SimTK::Vector tau(const SimTK::State& s, const SimTK::Vector& goal,
            const SimTK::Vector& tauP) const override;

    protected:

        TaskType taskType;

        std::string body;
        SimTK::Vec3 offset;

        SimTK::ReferencePtr<TaskTrackingController> controller;
    };

    /* PositionTask **********************************************************/

    /**
    * Implements a position controlled task, where the control law
    * is specified in the constructor (PD, etc.). The user specifies
    * a desired goal, which is tracked by the task, in a
    * closed loop manner.
    *
    * @author Dimitar Stanev
    */
    class TaskSpace_API PositionTask : public TaskPrimitive
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(PositionTask, TaskPrimitive);
    public:

        PositionTask(std::string body, SimTK::Vec3 offset,
            TaskTrackingController* controller = new TaskTrackingController());

        ~PositionTask();

        /**
        * Task position \f$ [x] \in R^{3 x 1} \f$
        */
        SimTK::Vector x(const SimTK::State& s) const override;

        /**
        * Task velocity \f$ [v] \in R^{3 x 1} \f$
        */
        SimTK::Vector v(const SimTK::State& s) const override;

        /**
        * Task acceleration \f$ [a] \in R^{3 x 1} \f$
        */
        SimTK::Vector a(const SimTK::State& s) const override;

        /**
        * Task Jacobian Task Jacobian \f$ J \in R^{3 x n} \f$
        */
        SimTK::Matrix J(const SimTK::State& s) const override;

        /**
        * Calculates the bias acceleration term \f$ \dot{J} \dot{q} \in R^{3 x 1} \f$
        */
        SimTK::Vector b(const SimTK::State& s) const override;
    };

    /* OrientationTask *******************************************************/

    /**
    * Implements a orientation task ([theta, omega, alpha]) which prescribes the
    * spatial orientation of the corresponding body, where the control law
    * is specified in the constructor (PD, etc.). The user specifies
    * a desired goal, which is tracked by the task, in a closed loop manner.
    *
    * @author Dimitar Stanev
    */
    class TaskSpace_API OrientationTask : public TaskPrimitive
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(OrientationTask, TaskPrimitive);
    public:

        OrientationTask(std::string body, SimTK::Vec3 offset = SimTK::Vec3(0),
            TaskTrackingController* controller = new TaskTrackingController());

        ~OrientationTask();

        /**
        * Task orientation \f$ [\theta] \in R^{3 x 1} \f$
        */
        SimTK::Vector x(const SimTK::State& s) const override;

        /**
        * Task angular velocity \f$ [w] \in R^{3 x 1} \f$

        */
        SimTK::Vector v(const SimTK::State& s) const override;

        /**
        * Task angular acceleration \f$ [aw] \in R^{3 x 1} \f$
        */
        SimTK::Vector a(const SimTK::State& s) const override;

        /**
        * Task Jacobian \f$ J \in R^{3 x n} \f$
        */
        SimTK::Matrix J(const SimTK::State& s) const override;

        /**
        * Calculates the bias acceleration term \f$ \dot{J} \dot{q} \in R^{3 x 1} \f$
        */
        SimTK::Vector b(const SimTK::State& s) const override;
    };

    /* SpatialTask ***********************************************************/

    /**
    * Implements a spatial task (orientation and position), where the control law
    * is specified in the constructor (PD, etc.). The user specifies
    * a desired goal, which is tracked by the task, in a closed loop manner.
    *
    * @author Dimitar Stanev
    */
    class TaskSpace_API SpatialTask : public TaskPrimitive
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(SpatialTask, TaskPrimitive);
    public:

        SpatialTask(std::string body, SimTK::Vec3 offset,
            TaskTrackingController* controller = new TaskTrackingController());

        ~SpatialTask();

        /**
        * Task position \f$ [x; \theta] \in R^{6 x 1} \f$
        */
        SimTK::Vector x(const SimTK::State& s) const override;

        /**
        * Task velocity \f$ [v; w] \in R^{6 x 1} \f$
        */
        SimTK::Vector v(const SimTK::State& s) const override;

        /**
        * Task acceleration \f$ [a; aw] \in R^{6 x 1} \f$
        */
        SimTK::Vector a(const SimTK::State& s) const override;

        /**
        * Task Jacobian \f$ J \in R^{6 x 1} \f$
        */
        SimTK::Matrix J(const SimTK::State& s) const override;

        /**
        * Calculates the bias acceleration term \f$ \dot{J} \dot{q} \in R^{6 x n} \f$
        */
        SimTK::Vector b(const SimTK::State& s) const override;
    };
}

#endif
