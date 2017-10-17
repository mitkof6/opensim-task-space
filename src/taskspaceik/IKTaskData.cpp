#include "IKTaskData.h"

#include <OpenSim/Simulation/Model/Model.h>

#include "TaskPrimitive.h"
#include "IKTrackingTask.h"

using namespace OpenSim;

IKTaskData::IKTaskData(std::string n)
    : name(n), task(NULL), trackingTask(NULL), kp(200), kv(20) {
}

IKTaskData::~IKTaskData() {
}

std::string IKTaskData::getName() const {
    return name;
}

TaskPrimitive* IKTaskData::updTask() {
    return task;
}

void IKTaskData::setTask(TaskPrimitive* t) {
    task = t;
}

double OpenSim::IKTaskData::getKp() const {
    return kp;
}

void OpenSim::IKTaskData::setKp(double k) {
    kp = k;
}

double OpenSim::IKTaskData::getKv() const {
    return kv;
}

void OpenSim::IKTaskData::setKv(double k) {
    kv = k;
}

void IKTaskData::addMarkerInfo(MarkerInfo mInfo) {
    markers.push_back(mInfo);
}

std::vector<IKTaskData::MarkerInfo>& IKTaskData::updMarkerInfo() {
    return markers;
}

IKTrackingTask* IKTaskData::updTrackingTask() {
    return trackingTask;
}

void IKTaskData::setTrackingTask(IKTrackingTask* t) {
    trackingTask = t;
}

const std::vector<SimTK::Transform>& IKTaskData::getTransitions() {
    return transitions;
}

void IKTaskData::setTransitions(const std::vector<SimTK::Transform>& t) {
    transitions = std::vector<SimTK::Transform>(t.begin(), t.end());
}

std::vector<int> IKTaskData::getCoordinateInicies() {
    std::vector<int> inx;

    for (auto marker : markers) {
        inx.push_back(marker.coordinateIndex);
    }

    return inx;
}

void IKTaskData::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    model.addComponent(trackingTask);
}
