#ifndef TASK_DYNAMICS_H
#define TASK_DYNAMICS_H

#include "internal/TaskSpaceDLL.h"

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Storage.h>
#include "ConstraintModel.h"

namespace OpenSim {
    /* TaskDynamics ***********************************************************/

    // 
    class DynamicCompensator;
    class TaskPriorityGraph;
    class AbstractPrimitive;

    /**
     * A task manager, which is responsible for computing the task torques and
     * compensation terms. This class is intended to be used as a base class for
     * the implementation of different control schemes, such as prioritization
     * control.
     *
     * @author Dimitar Stanev
     */
    class TaskSpace_API TaskDynamics : public ModelComponent {
        OpenSim_DECLARE_ABSTRACT_OBJECT(TaskDynamics, ModelComponent);
    public:

        TaskDynamics(ConstraintModel::Type type);

        virtual ~TaskDynamics();

        /**
         * The task component is also added to the model, which takes ownership
         * of the task.
         */
        void addTask(AbstractPrimitive* taskPrimitive, std::string name,
		     std::string parent);

        /**
         * Calculates the total task torques if task = NULL or, the contribution
         * of the individual task.
         */
        virtual SimTK::Vector calcTaskTorques(const SimTK::State& s) = 0;

        const DynamicCompensator& getDynamicCompensator() const;
        DynamicCompensator& updDynamicCompensator();

        void printResults(std::string prefix, std::string dir);

        void printPriorityGraph() const;

        /**
         * Active joints selection matrix
         */
        SimTK::Matrix B(const SimTK::State& s) const;

    protected:

        void appendAnalytics(const SimTK::State& s,
                             const SimTK::Vector& taskTorques,
                             const SimTK::Vector& constraintTorques,
                             const SimTK::Vector& residualTorques,
                             const SimTK::Vector& lambda);

        // ModelComponent methods

        void extendAddToSystem(SimTK::MultibodySystem& system) const override;

        void extendConnectToModel(Model& model) override;

        // TODO find shortcut to initialize after model has been realized
        void initializeFromState(const SimTK::State& s) const;
        mutable bool initialized = false;

    protected:

        // data members

        ConstraintModel* constraintModel;
        DynamicCompensator* compensator;
        TaskPriorityGraph* taskGraph;
        mutable Storage analytics;
        static const std::string CACHE_B;
    };

    /* TaskDynamicsPrioritization *********************************************/

    /**
     * Implements a prioritization task scheme.
     *
     * @author Dimitar Stanev
     */
    class TaskSpace_API TaskDynamicsPrioritization : public TaskDynamics {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskDynamicsPrioritization, TaskDynamics);
    public:

        TaskDynamicsPrioritization(ConstraintModel::Type type);

        ~TaskDynamicsPrioritization();

        /**
         * Calculates the total task torques to achieve the desired goal.
         */
        SimTK::Vector calcTaskTorques(const SimTK::State& s) override;
    };
}

#endif
