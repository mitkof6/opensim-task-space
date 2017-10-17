#include "StationPointAnalysis.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

StationPointAnalysis::StationPointAnalysis(Model* model, string name, Vec3 offset)
    : Analysis(model), bodyName(name), markerOffset(offset)
{
    constructDescription();
    constructColumnLabels();
    setupStorage();

    setName("StationPointAnalysis");
}

void StationPointAnalysis::setModel(Model& aModel)
{
    Super::setModel(aModel);

    constructDescription();
    constructColumnLabels();
    setupStorage();
}

int StationPointAnalysis::begin(SimTK::State& s)
{
    if (!proceed()) return(0);

    storage.reset(s.getTime());

    int status = 0;
    if (storage.getSize() <= 0)
    {
        status = record(s);
    }

    return(status);
}

int StationPointAnalysis::step(const SimTK::State& s, int stepNumber)
{
    if (!proceed(stepNumber)) return(0);

    record(s);

    return(0);
}

int StationPointAnalysis::end(SimTK::State& s)
{
    if (!proceed()) return(0);

    record(s);

    return(0);
}

int StationPointAnalysis::print(const std::string &path)
{
    storage.print(path);

    return(0);
}

int StationPointAnalysis::record(const SimTK::State& s)
{
    //_model->getMultibodySystem().realize(s, SimTK::Stage::Report);

    //append storage
    Array<double> data;
    Vec3 temp;

    _model->getSimbodyEngine().getPosition(s, _model->getBodySet().get(bodyName),
                                           markerOffset, temp);
    data.append(temp[0]);
    data.append(temp[1]);
    data.append(temp[2]);

    _model->getSimbodyEngine().getVelocity(s, _model->getBodySet().get(bodyName),
                                           markerOffset, temp);
    data.append(temp[0]);
    data.append(temp[1]);
    data.append(temp[2]);

    _model->getSimbodyEngine().getAcceleration(s, _model->getBodySet().get(bodyName),
                                               markerOffset, temp);
    data.append(temp[0]);
    data.append(temp[1]);
    data.append(temp[2]);

    storage.append(s.getTime(), data.size(), &data[0]);

    return(0);
}

void StationPointAnalysis::constructDescription()
{
    string descrip;

    descrip = "\nThis file contains the record of custom variables\n";
    descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
    if (getInDegrees())
    {
        descrip += "\nAngles are in degrees.";
    }
    else
    {
        descrip += "\nAngles are in radians.";
    }
    descrip += "\n\n";

    setDescription(descrip);
}

void StationPointAnalysis::constructColumnLabels()
{
    if (_model == NULL) return;

    Array<string> labels;
    labels.append("time");
    labels.append("position_x");
    labels.append("position_y");
    labels.append("position_z");
    labels.append("velocity_x");
    labels.append("velocity_y");
    labels.append("velocity_z");
    labels.append("acceleration_x");
    labels.append("acceleration_y");
    labels.append("acceleration_z");

    setColumnLabels(labels);
}

void StationPointAnalysis::setupStorage()
{
    storage.reset(0);
    storage.setName("StationPointAnalysis");
    storage.setDescription(getDescription());
    storage.setColumnLabels(getColumnLabels());
}

int StationPointAnalysis::printResults(const string &aBaseName,
                                       const string &aDir, double aDT,
				       const string &aExtension)
{
    Storage::printResult(&storage, aBaseName + "_" + getName(), aDir, aDT, aExtension);

    return(0);
}

void StationPointAnalysis::getStationState(double t, Vec3& pos, Vec3& vel,
					   Vec3& acc) const
{
    Array<double> state(0.0, 9);
    int flag = storage.getDataAtTime(t, 9, state);

    if (!flag)
    {
        throw Exception("The storage is empty\n", __FILE__, __LINE__);
    }

    pos = Vec3(state[0], state[1], state[2]);
    vel = Vec3(state[3], state[4], state[5]);
    acc = Vec3(state[6], state[7], state[8]);
}

Storage& StationPointAnalysis::getStorage()
{
    return storage;
}
