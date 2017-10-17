#include "MarkerErrorAnalysis.h"

#include <OpenSim/Simulation/Model/Model.h>

#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace std;

MarkerErrorAnalysis::MarkerErrorAnalysis(Model* model, const MarkerData& markerData)
    : Analysis(model), markerData(markerData)
{
    constructDescription();
    constructColumnLabels();
    setupStorage();

    setName("MarkerError");
}

void MarkerErrorAnalysis::setModel(Model& aModel)
{
    Super::setModel(aModel);

    constructDescription();
    constructColumnLabels();
    setupStorage();
}

int MarkerErrorAnalysis::begin(SimTK::State& s)
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

int MarkerErrorAnalysis::step(const SimTK::State& s, int stepNumber)
{
    if (!proceed(stepNumber)) return(0);

    record(s);

    return(0);
}

int MarkerErrorAnalysis::end(SimTK::State& s)
{
    if (!proceed()) return(0);

    record(s);

    return(0);
}

int MarkerErrorAnalysis::print(const std::string &path)
{
    storage.print(path);

    return(0);
}

int MarkerErrorAnalysis::record(const SimTK::State& s)
{
    Array<double> markerErrorData;
    int frameStart, frameEnd;
    markerData.findFrameRange(s.getTime(), s.getTime(), frameStart, frameEnd);

    double sumDist = 0;
    int n = 0;
    double min = SimTK::Infinity;
    double max = 0;

    const MarkerSet& ms = _model->getMarkerSet();
    for (int i = 0; i < ms.getSize(); i++)
    {
	SimTK::Vec3 xVM = ms[i].findLocationInFrame(s, _model->getGround());
        SimTK::Vec3 xOM = markerData.getFrame(frameStart).getMarker(
            markerData.getMarkerIndex(ms[i].getName()));

        double distSq = distanceSq(xVM, xOM);
        markerErrorData.append(sqrt(distSq));

        if (distSq > max)
        {
            max = distSq;
        }

        if (distSq < min)
        {
            min = distSq;
        }

        sumDist += distSq;
        n++;
    }

    rms = sqrt(sumDist / n);

    markerErrorData.append(sqrt(min));
    markerErrorData.append(sqrt(max));
    markerErrorData.append(sumDist);
    markerErrorData.append(rms);

    storage.append(s.getTime(), markerErrorData.size(), &markerErrorData[0], true);

    return(0);
}

void MarkerErrorAnalysis::constructDescription()
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

void MarkerErrorAnalysis::constructColumnLabels()
{
    if (_model == NULL) return;

    Array<string> labels;
    labels.append("time");
    Array<string> markerNames = markerData.getMarkerNames();

    auto ms = _model->getMarkerSet();
    for (int i = 0; i < ms.getSize(); i++)
    {
        string name = ms[i].getName();
        for (int j = 0; j < markerNames.size(); j++)
        {
            if (name == markerNames[j])
            {
                labels.append(name);
                break;
            }
        }
    }

    labels.append("min");
    labels.append("max");
    labels.append("TotalSq");
    labels.append("RMS");

    setColumnLabels(labels);
}

void MarkerErrorAnalysis::setupStorage()
{
    storage.reset(0);
    storage.setName(getName());
    storage.setDescription(getDescription());
    storage.setColumnLabels(getColumnLabels());
}

int MarkerErrorAnalysis::printResults(const string &aBaseName,
    const string &aDir, double aDT, const string &aExtension)
{
    Storage::printResult(&storage, aBaseName + "_" + getName(), aDir, aDT, aExtension);

    return(0);
}

Storage& MarkerErrorAnalysis::getStorage()
{
    return storage;
}

double MarkerErrorAnalysis::getCurrentRMS() const
{
    return rms;
}
