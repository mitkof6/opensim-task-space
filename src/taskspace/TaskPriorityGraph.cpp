#include "TaskPriorityGraph.h"

#include <OpenSim/Simulation/Model/Model.h>

#include "AbstractPrimitive.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

/* TaskPriorityGraph **********************************************************/

TaskPriorityGraph::TaskPriorityGraph() {
}

TaskPriorityGraph::~TaskPriorityGraph() {
}

void TaskPriorityGraph::addTask(AbstractPrimitive* taskPrimitive, string name,
                                string parent) {
    TaskData* task = new TaskData();
    task->name = name;
    task->task = taskPrimitive;

    if (parent == "") {// root
        if (taskTree.size() != 0) {
            throw OpenSim::Exception(
                "Task tree can't contain multiple parent nodes",
                __FILE__, __LINE__);
        }

        taskTree.insert(taskTree.begin(), task);
        cout << "Task: " << task->name << " added as root" << endl;
    }
    else {
        auto it = findByName(taskTree, parent);
        if (it != taskTree.end()) {
            taskTree.append_child(it, task);
            cout << "Task: " << task->name << " added with parent: "
                 << parent << endl;
        }
        else {
            throw OpenSim::Exception(
                "Task tree does not contain a task with name: "
                + parent, __FILE__, __LINE__);
        }
    }
}

tree<TaskPriorityGraph::TaskData*>& TaskPriorityGraph::updGraph() {
    return taskTree;
}

void TaskPriorityGraph::print() const {
    tree<TaskData*>::pre_order_iterator it = taskTree.begin();
    tree<TaskData*>::pre_order_iterator end = taskTree.end();
    cout << "Task Tree Hierarchy:" << endl;
    while (it != end) {
        //cout << bodyTree.depth(it) << endl;
        for (int i = 0; i < taskTree.depth(it); i++) {
            cout << "\t";
        }
        cout << it.node->data->name << endl;
        ++it;
    }
}

tree<TaskPriorityGraph::TaskData*>::iterator TaskPriorityGraph::findByName(
    const tree<TaskPriorityGraph::TaskData*>& taskTree, std::string name) {
    tree<TaskData*>::iterator it = taskTree.begin();
    while (it != taskTree.end()) {
        if (name == (*it)->name) {
            return it;
        }
        ++it;
    }
    return taskTree.end();
}

void TaskPriorityGraph::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    for (auto t : taskTree) {
        model.addModelComponent(t->task);
    }
}
