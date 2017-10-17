#include "MarkerPositionAnalysis.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using namespace std;

MarkerPositionAnalysis::MarkerPositionAnalysis(Model* model)
    : Analysis(model)
{
    constructDescription();
    constructColumnLabels();
    setupStorage();

    setName("MarkerPosition");
}

void MarkerPositionAnalysis::setModel(Model& aModel)
{
    Super::setModel(aModel);

    constructDescription();
    constructColumnLabels();
    setupStorage();
}

int MarkerPositionAnalysis::begin(SimTK::State& s)
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

int MarkerPositionAnalysis::step(const SimTK::State& s, int stepNumber)
{
    if (!proceed(stepNumber)) return(0);

    record(s);

    return(0);
}

int MarkerPositionAnalysis::end(SimTK::State& s)
{
    if (!proceed()) return(0);

    record(s);

    return(0);
}

int MarkerPositionAnalysis::print(const std::string &path)
{
    storage.print(path);

    return(0);
}

int MarkerPositionAnalysis::record(const SimTK::State& s)
{
    //append storage
    Array<double> data;
    SimTK::Vec3 temp;

    const MarkerSet& ms = _model->getMarkerSet();
    for (int i = 0; i < ms.getSize(); i++)
    {
	SimTK::Vec3 pos = ms[i].findLocationInFrame(s, _model->getGround());
        data.append(pos[0]);
        data.append(pos[1]);
        data.append(pos[2]);
    }

    storage.append(s.getTime(), data.size(), &data[0], true);

    return(0);
}

void MarkerPositionAnalysis::constructDescription()
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

void MarkerPositionAnalysis::constructColumnLabels()
{
    if (_model == NULL) return;

    Array<string> labels;
    labels.append("time");

    auto ms = _model->getMarkerSet();
    for (int i = 0; i < ms.getSize(); i++)
    {
        string name = ms[i].getName();
        labels.append(name + "_x");
        labels.append(name + "_y");
        labels.append(name + "_z");
    }

    setColumnLabels(labels);
}

void MarkerPositionAnalysis::setupStorage()
{
    storage.reset(0);
    storage.setName(getName());
    storage.setDescription(getDescription());
    storage.setColumnLabels(getColumnLabels());
}

int MarkerPositionAnalysis::printResults(const string &aBaseName,
    const string &aDir, double aDT, const string &aExtension)
{
    Storage::printResult(&storage, aBaseName + "_" + getName(), aDir, aDT, aExtension);

    return(0);
}

Storage& MarkerPositionAnalysis::getStorage()
{
    return storage;
}
