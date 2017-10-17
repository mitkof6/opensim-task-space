#include "IKTaskManager.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>

#include "TaskDynamics.h"
#include "TaskPrimitive.h"
#include "TaskTrackingController.h"
#include "IKTaskData.h"
#include "IKTrackingTask.h"
#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace std;

IKTaskManager::IKTaskManager(ConstraintModel::Type type, double t0,
    const MarkerData& mD, bool filter, bool v)
    : verbose(v), maxRMS(0), startTime(t0), markerData(mD) {
    constructProperty_linearResampling(0.01);
    constructProperty_cutOffFrequencty(6);
    constructProperty_iirOrder(50);
    constructProperty_splineOrder(5);

    taskDynamics = new TaskDynamicsPrioritization(type);

    markerData.convertToUnits(Units::Meters);
    markerNames = markerData.getMarkerNames();

    // prepare markerDataStorage
    if (verbose) {
        cout << "Marker data resumpled at: " << get_linearResampling() << endl;
        cout << "Marker data filtered with IIR order: " << get_iirOrder()
            << " cut off frequency: " << get_cutOffFrequencty() << "Hz" << endl;
        cout << "Marker data smooth splined with order: " << get_splineOrder()
            << " cut off frequency: " << get_cutOffFrequencty() << "Hz" << endl;
    }

    markerData.makeRdStorage(markerDataStorage);
    if (filter) {
        markerDataStorage.resampleLinear(get_linearResampling());
        markerDataStorage.pad(markerDataStorage.getSize() / 2);
        markerDataStorage.lowpassFIR(get_iirOrder(), get_cutOffFrequencty());
        markerDataStorage.smoothSpline(get_splineOrder(), get_cutOffFrequencty());
    }
    markerSplineSet = GCVSplineSet(get_splineOrder(), &markerDataStorage);
}

IKTaskManager::~IKTaskManager() {
}

void IKTaskManager::initializeFromState(SimTK::State& s) {
    // perform depth first iteration and calculate transitions
    _model->realizePosition(s);
    for (tree<IKTaskData*>::pre_order_iterator node = bodyTree.begin()
        ; node != bodyTree.end(); ++node) {
        SimTK::Transform T0 = _model->getBodySet()
            .get(node.node->data->getName()).getGroundTransform(s);
        node.node->data->updTrackingTask()->calcTransitions(T0, *node.node);
    }
}

void IKTaskManager::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    constructTasksFromMarkers();
    constructTrackingTasks();

    // construct priority tree
    for (tree<IKTaskData*>::pre_order_iterator it = bodyTree.begin();
        it != bodyTree.end(); it++) {
        if (it == bodyTree.begin()) {
            taskDynamics->addTask(it.node->data->updTask(), it.node->data->getName(), "");
        }
        else {
            auto& parent = it.node->parent;
            taskDynamics->addTask(it.node->data->updTask(), it.node->data->getName(),
                parent->data->getName());
        }
    }

    if (verbose) {
        taskDynamics->printPriorityGraph();
    }

    model.addModelComponent(taskDynamics);
}

void IKTaskManager::constructTasksFromMarkers() {
    MarkerSet& ms = _model->updMarkerSet();

    if (ms.getSize() == 0) {
        throw OpenSim::Exception("The model does not have any markers",
            __FILE__, __LINE__);
    }

    // construct body tree
    const JointSet& js = _model->getJointSet();
    string floatingBase = "";
    for (int i = 0; i < js.getSize(); i++) {
        Joint& joint = js[i];

        // correct naming convention
        string parent = joint.getParentFrame().getName();
        if (parent.find("_offset") != std::string::npos) {
            parent = parent.substr(0, parent.length() - 7);
        }
        string child = joint.getChildFrame().getName();
        if (child.find("_offset") != std::string::npos) {
            child = child.substr(0, child.length() - 7);
        }

        if (parent == "ground") {
            auto root = bodyTree.begin();
            auto node = bodyTree.insert(root, new IKTaskData(child));

            // check if this is a floating base
            if (joint.numCoordinates() == 6) {
                floatingBase = joint.getChildFrame().getName();

                if (verbose) {
                    cout << "Found floating base: " << floatingBase << endl;
                }
            }

            if (verbose) {
                printTree();
            }
        }
        else {
            auto parentNode = find(parent);
            if (parentNode != bodyTree.end()) {
                auto node = bodyTree.append_child(parentNode, new IKTaskData(child));
            }
            else {
                throw OpenSim::Exception("Parent node: " + parent + " with child: " +
                    child + " doesn't exist.",
                    __FILE__, __LINE__);
            }
        }
    }

    // populate marker error labels
    Array<string> markerErrorLabels;
    markerErrorLabels.append("time");

    // group markers per body
    for (int i = 0; i < ms.getSize(); i++) {
        string name = ms[i].getName();
        string body = ms[i].getFrameName();

        // check if the marker exists within the recorded motion
        bool found = false;
        int index;
        for (int j = 0; j < markerNames.getSize(); j++) {
            if (markerNames[j] == name) {
                found = true;
                index = j;
                break;
            }
        }
        if (!found) {
            if (verbose) {
                cout << "Marker: " << name << " does not exist in .trc" << endl;
            }
            continue;
        }

        auto node = find(body);
        if (node != bodyTree.end()) {
            node.node->data->addMarkerInfo(
                IKTaskData::MarkerInfo(index, &ms[i]));

            if (verbose) {
                cout << "Marker: " << name << " assigned to body: " << body << endl;
            }

            markerErrorLabels.append(name);
        }
        else {
            throw OpenSim::Exception("Node: " + body + " doesn't exist.",
                __FILE__, __LINE__);
        }
    }

    // construct marker error labels
    markerErrorLabels.append("min");
    markerErrorLabels.append("max");
    markerErrorLabels.append("TotalSq");
    markerErrorLabels.append("RMS");
    markerError.setColumnLabels(markerErrorLabels);
    desiredCommands.reset(0);

    // populate desired command labels
    Array<string> desiredCommandLabels;
    desiredCommandLabels.append("time");

    // TODO construct based on degrees of freedom and not based on markers (not necessary)
    // construct tasks from marker groups
    vector<string> invalideNodes;
    for (auto data : bodyTree) {
        if (floatingBase != "" && floatingBase == data->getName()) {// handle special case when there is a floating base
            Marker* marker = data->updMarkerInfo()[0].marker;
            string body = marker->getFrameName();
            SimTK::Vec3 offset(0);

            TaskPrimitive* task = new SpatialTask(body, offset);
            task->setName(body);
            data->setTask(task);

            if (verbose) {
                cout << "Spatial task: " << task->getName() << " "
                    << task->getOffset() << " " << task->getBody() << endl;
            }

            appendLabelsForTask(desiredCommandLabels, task);
        }
        else {
            if (data->updMarkerInfo().size() == 1) {// if a body has only one or two markers (ignores the second)
                Marker* marker = data->updMarkerInfo()[0].marker;
                SimTK::Vec3 offset = marker->get_location();
                string body = marker->getFrameName();

                TaskPrimitive* task = new PositionTask(body, offset);
                task->setName(body);
                data->setTask(task);

                if (verbose) {
                    cout << "Position task: " << task->getName() << " "
                        << task->getOffset() << " " << task->getBody() << endl;
                }

                if (verbose && data->updMarkerInfo().size() == 2) {
                    cout << "Body: " << body << " has 2 markers and only "
                        << marker->getName() << " is used" << endl;
                }

                appendLabelsForTask(desiredCommandLabels, data->updTask());
            }
            else if (data->updMarkerInfo().size() > 1) {// if a body has three or more markers
                string body = data->getName();
                SimTK::Vec3 offset(0);

                // TODO properly

                /*TaskPrimitive* task = new OrientationTask(body, offset);
                task->setName(body);
                data->setTask(task);

                if (verbose)
                {
                    cout << "Orientation task: " << task->getName() << endl;
                }*/

                TaskPrimitive* task = new SpatialTask(body, offset);
                task->setName(body);
                data->setTask(task);

                if (verbose) {
                    cout << "Spatial task: " << task->getName() << endl;
                }

                appendLabelsForTask(desiredCommandLabels, task);
            }
            else {
                invalideNodes.push_back(data->getName());
                bodyTree.flatten(find(data->getName()));

                if (verbose) {
                    cout << "Body: " + data->getName() << " does not have markers and will be"
                        " marked as invalid node (deleted later)" << endl;

                    //IKTaskData::printTree(bodyTree);
                }
            }
        }
    }

    // construct desired commands labels
    desiredCommands.setColumnLabels(desiredCommandLabels);
    desiredCommands.reset(0);

    for (auto invalideNode : invalideNodes) {
        auto it = find(invalideNode);
        bodyTree.erase(it);
    }

    if (verbose) {
        printTree();
    }
}

void IKTaskManager::constructTrackingTasks() {
    Array<double> t = getTimeVector();

    for (auto data : bodyTree) {
        vector<int> idx = data->getCoordinateInicies();

        if (data->updTask()->getTaskType() == TaskPrimitive::POSITION) {
            IKTrackingTask* trackingTask = new IKPositionTrackingTask(t,
                &markerSplineSet[3 * idx[0]],
                &markerSplineSet[3 * idx[0] + 1],
                &markerSplineSet[3 * idx[0] + 2]);
            data->setTrackingTask(trackingTask);
        }
        else if (data->updTask()->getTaskType() == TaskPrimitive::SPATIAL) {
            IKTrackingTask* trackingTask = new IKSpatialTrackingTask(t,
                markerSplineSet, idx);
            data->setTrackingTask(trackingTask);
        }
        else if (data->updTask()->getTaskType() == TaskPrimitive::ORIENTATION) {
            IKTrackingTask* trackingTask = new IKOrientationTrackingTask(t,
                markerSplineSet, idx);
            data->setTrackingTask(trackingTask);
        }
        else {
            throw OpenSim::Exception("Task type: " +
                data->updTask()->getPrimitiveTypeAsString() +
                " is not supported", __FILE__, __LINE__);
        }
    }
}

void IKTaskManager::updateGoals(const SimTK::State& s, double td) const {
    _model->realizeVelocity(s);

    double rms = calcMarkerError(s);
    if (rms > maxRMS) {
        maxRMS = rms;
    }

    if (verbose) {
        cout << "Total RMS: " << rms << " at: " << s.getTime() << "s" << endl;

        cout << "Update goals for target time: " << td << endl;
    }

    Array<double> goalData;
    for (auto data : bodyTree) {
        TaskPrimitive* task = data->updTask();
        IKTrackingTask* trackingTask = data->updTrackingTask();

        SimTK::Vector xd, vd, ad;
        trackingTask->calcGoal(td, xd, vd, ad);

        SimTK::Vector goal = 1 * ad
            - data->getKp() * (task->x(s) - xd)
            - data->getKv() * (task->v(s) - vd);
        task->setGoal(s, goal);

        populateDesiredCommand(goalData, xd, vd, ad, task);

        if (0 && verbose) {
            cout << "Body: " << data->getName() << endl;
            cout << "x: " << toDeg(task->x(s), 0, 2) << endl;
            cout << "xd: " << toDeg(xd, 0, 2) << endl;
            cout << "v: " << task->v(s) << endl;
            cout << "vd: " << vd << endl;
            cout << "a: " << task->a(s) << endl;
            cout << "ad: " << ad << endl;
            cout << "Goal: " << goal << endl;
        }
    }

    // insert desired commands
    desiredCommands.append(td, goalData.size(), &goalData[0]);
}

void IKTaskManager::printResults(std::string prefix, std::string dir) {
    if (verbose) {
        cout << "Max RMS: " << maxRMS << endl;
    }

    desiredCommands.print(dir + "/" + prefix + "_DesiredCommands.sto");
    markerError.print(dir + "/" + prefix + "_MarkerError.sto");
}

MarkerData& IKTaskManager::getMarkerData() {
    return markerData;
}

Array<double> IKTaskManager::getTimeVector() {
    Array<double> t;
    markerDataStorage.getTimeColumnWithStartTime(t, startTime);

    return t;
}

TaskDynamics& IKTaskManager::updTaskDynamics() {
    return *taskDynamics;
}

double IKTaskManager::calcMarkerError(const SimTK::State& s) const {
    Array<double> markerErrorData;

    double sumDist = 0;
    int n = 0;
    double min = SimTK::Infinity;
    double max = 0;

    for (auto data : bodyTree) {
        for (auto marker : data->updMarkerInfo()) {
            SimTK::Vec3 xVM = marker.marker->findLocationInFrame(s, _model->getGround());
            SimTK::Vec3 xOM(
                markerSplineSet[3 * marker.coordinateIndex].calcValue(SimTK::Vector(1, s.getTime())),
                markerSplineSet[3 * marker.coordinateIndex + 1].calcValue(SimTK::Vector(1, s.getTime())),
                markerSplineSet[3 * marker.coordinateIndex + 2].calcValue(SimTK::Vector(1, s.getTime())));

            double distSq = distanceSq(xVM, xOM);
            markerErrorData.append(sqrt(distSq));

            if (distSq > max) {
                max = distSq;
            }

            if (distSq < min) {
                min = distSq;
            }

            sumDist += distSq;
            n++;
        }
    }

    double rms = sqrt(sumDist / n);

    markerErrorData.append(sqrt(min));
    markerErrorData.append(sqrt(max));
    markerErrorData.append(sumDist);
    markerErrorData.append(rms);
    markerError.append(s.getTime(), markerErrorData.size(), &markerErrorData[0]);

    return rms;
}

void IKTaskManager::appendLabelsForTask(Array<string>& labels, const TaskPrimitive* task) {
    if (task->getTaskType() == TaskPrimitive::TaskType::SPATIAL) {
        labels.append(task->getName() + string("_thx"));
        labels.append(task->getName() + string("_thy"));
        labels.append(task->getName() + string("_thz"));
        labels.append(task->getName() + string("_px"));
        labels.append(task->getName() + string("_py"));
        labels.append(task->getName() + string("_pz"));

        labels.append(task->getName() + string("_wx"));
        labels.append(task->getName() + string("_wy"));
        labels.append(task->getName() + string("_wz"));
        labels.append(task->getName() + string("_vx"));
        labels.append(task->getName() + string("_vy"));
        labels.append(task->getName() + string("_vz"));

        labels.append(task->getName() + string("_alx"));
        labels.append(task->getName() + string("_aly"));
        labels.append(task->getName() + string("_alz"));
        labels.append(task->getName() + string("_ax"));
        labels.append(task->getName() + string("_ay"));
        labels.append(task->getName() + string("_az"));
    }
    else if (task->getTaskType() == TaskPrimitive::TaskType::ORIENTATION) {
        labels.append(task->getName() + string("_thx"));
        labels.append(task->getName() + string("_thy"));
        labels.append(task->getName() + string("_thz"));

        labels.append(task->getName() + string("_wx"));
        labels.append(task->getName() + string("_wy"));
        labels.append(task->getName() + string("_wz"));

        labels.append(task->getName() + string("_alx"));
        labels.append(task->getName() + string("_aly"));
        labels.append(task->getName() + string("_alz"));
    }
    else if (task->getTaskType() == TaskPrimitive::TaskType::POSITION) {
        labels.append(task->getName() + string("_px"));
        labels.append(task->getName() + string("_py"));
        labels.append(task->getName() + string("_pz"));

        labels.append(task->getName() + string("_vx"));
        labels.append(task->getName() + string("_vy"));
        labels.append(task->getName() + string("_vz"));

        labels.append(task->getName() + string("_ax"));
        labels.append(task->getName() + string("_ay"));
        labels.append(task->getName() + string("_az"));
    }
}

void IKTaskManager::populateDesiredCommand(Array<double>& goalData, const SimTK::Vector& xd,
    const SimTK::Vector& vd, const SimTK::Vector& ad, const TaskPrimitive* task) const {
    // append desired command
    for (int i = 0; i < xd.size(); i++) {
        if (task->getTaskType() != TaskPrimitive::TaskType::POSITION && i < 3) {
            goalData.append(SimTK::convertRadiansToDegrees(xd[i]));
        }
        else {
            goalData.append(xd[i]);
        }
    }
    for (int i = 0; i < vd.size(); i++) {
        if (task->getTaskType() != TaskPrimitive::TaskType::POSITION && i < 3) {
            goalData.append(SimTK::convertRadiansToDegrees(vd[i]));
        }
        else {
            goalData.append(vd[i]);
        }
    }
    for (int i = 0; i < ad.size(); i++) {
        if (task->getTaskType() != TaskPrimitive::TaskType::POSITION && i < 3) {
            goalData.append(SimTK::convertRadiansToDegrees(ad[i]));
        }
        else {
            goalData.append(ad[i]);
        }
    }
}

tree<IKTaskData*>::iterator IKTaskManager::find(const std::string& name) {
    auto it = bodyTree.begin();
    while (it != bodyTree.end()) {
        if (name == (*it)->getName()) {
            return it;
        }
        ++it;
    }
    return bodyTree.end();
}

void IKTaskManager::printTree() {
    auto it = bodyTree.begin();
    auto end = bodyTree.end();
    std::cout << "Tree Hierarchy:" << std::endl;
    while (it != end) {
        for (int i = 0; i < bodyTree.depth(it); i++) {
            std::cout << "\t";
        }
        std::cout << it.node->data->getName() << std::endl;
        ++it;
    }
}