#ifndef TASK_PRIORITY_GRAPH_H
#define TASK_PRIORITY_GRAPH_H

#include "internal/TaskSpaceDLL.h"

#include <OpenSim/Simulation/Model/ModelComponent.h>

#include "tree.h"

namespace OpenSim
{
    /* TaskPriorityGraph ******************************************************************/

    //forward declaration
    class AbstractPrimitive;

    /**
    *
    *
    * @author Dimitar Stanev
    */
    class TaskSpace_API TaskPriorityGraph : public ModelComponent
    {
        OpenSim_DECLARE_CONCRETE_OBJECT(TaskPriorityGraph, ModelComponent);
    public:

        struct TaskData
        {
            std::string name;
            AbstractPrimitive* task;
            SimTK::Vector tauP;
            SimTK::Matrix NpT;
        };

        TaskPriorityGraph();

        ~TaskPriorityGraph();

        void addTask(AbstractPrimitive* taskPrimitive, std::string name,
            std::string parent);

        tree<TaskData*>& updGraph();

        void print() const;

        static tree<TaskData*>::iterator findByName(
            const tree<TaskData*>& taskTree, std::string name);

    protected:

        // ModelComponent methods

        void extendConnectToModel(Model& model) override;

    private:

        tree<TaskData*> taskTree;
    };
}

#endif